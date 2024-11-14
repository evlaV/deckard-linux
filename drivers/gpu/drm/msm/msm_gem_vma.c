// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2016 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 */

#include "msm_drv.h"
#include "msm_gem.h"
#include "msm_mmu.h"

static void
msm_gem_vm_free(struct drm_gpuvm *gpuvm)
{
	struct msm_gem_vm *vm = container_of(gpuvm, struct msm_gem_vm, base);

	drm_mm_takedown(&vm->mm);
	if (vm->mmu)
		vm->mmu->funcs->destroy(vm->mmu);
	put_pid(vm->pid);
	kfree(vm);
}

/* Actually unmap memory for the vma */
void msm_gem_vma_purge(struct drm_gpuva *vma)
{
	struct msm_gem_vma *msm_vma = to_msm_vma(vma);
	struct msm_gem_vm *vm = to_msm_vm(vma->vm);
	unsigned size = vma->va.range;

	/* Don't do anything if the memory isn't mapped */
	if (!msm_vma->mapped)
		return;

	vm->mmu->funcs->unmap(vm->mmu, vma->va.addr, size);

	msm_vma->mapped = false;
}

/* Map and pin vma: */
int
msm_gem_vma_map(struct drm_gpuva *vma, int prot, struct sg_table *sgt)
{
	struct msm_gem_vma *msm_vma = to_msm_vma(vma);
	struct msm_gem_vm *vm = to_msm_vm(vma->vm);
	int ret;

	if (GEM_WARN_ON(!vma->va.addr))
		return -EINVAL;

	if (msm_vma->mapped)
		return 0;

	msm_vma->mapped = true;

	/*
	 * NOTE: iommu/io-pgtable can allocate pages, so we cannot hold
	 * a lock across map/unmap which is also used in the job_run()
	 * path, as this can cause deadlock in job_run() vs shrinker/
	 * reclaim.
	 *
	 * Revisit this if we can come up with a scheme to pre-alloc pages
	 * for the pgtable in map/unmap ops.
	 */
	ret = vm->mmu->funcs->map(vm->mmu, vma->va.addr, sgt,
				  vma->gem.offset, vma->va.range,
				  prot);
	if (ret) {
		msm_vma->mapped = false;
	}

	return ret;
}

/* Close an iova.  Warn if it is still in use */
void msm_gem_vma_close(struct drm_gpuva *vma)
{
	struct msm_gem_vm *vm = to_msm_vm(vma->vm);
	struct msm_gem_vma *msm_vma = to_msm_vma(vma);

	GEM_WARN_ON(msm_vma->mapped);

	spin_lock(&vm->mm_lock);
	if (vma->va.addr && vm->managed)
		drm_mm_remove_node(&msm_vma->node);
	spin_unlock(&vm->mm_lock);

	dma_resv_lock(drm_gpuvm_resv(vma->vm), NULL);
	mutex_lock(&vm->vm_lock);
	drm_gpuva_remove(vma);
	drm_gpuva_unlink(vma);
	mutex_unlock(&vm->vm_lock);
	dma_resv_unlock(drm_gpuvm_resv(vma->vm));

	kfree(vma);
}

/* Create a new vma and allocate an iova for it */
struct drm_gpuva *
msm_gem_vma_new(struct drm_gpuvm *_vm, struct drm_gem_object *obj,
		uint32_t align, u64 offset, u64 range_start, u64 range_end)
{
	struct msm_gem_vm *vm = to_msm_vm(_vm);
	struct drm_gpuvm_bo *vm_bo;
	struct msm_gem_vma *vma;
	int ret;

	vma = kzalloc(sizeof(*vma), GFP_KERNEL);
	if (!vma)
		return ERR_PTR(-ENOMEM);

	if (vm->managed) {
		BUG_ON(offset != 0);
		spin_lock(&vm->mm_lock);
		ret = drm_mm_insert_node_in_range(&vm->mm, &vma->node,
						obj->size, max(1ul << align, PAGE_SIZE), 0,
						range_start, range_end, 0);
		spin_unlock(&vm->mm_lock);

		if (ret)
			goto err_free_vma;

		range_start = vma->node.start;
		range_end   = range_start + obj->size;
	}

	GEM_WARN_ON((range_end - range_start) > obj->size);

	drm_gpuva_init(&vma->base, range_start, range_end - range_start, obj, offset);
	vma->mapped = false;

	mutex_lock(&vm->vm_lock);
	ret = drm_gpuva_insert(&vm->base, &vma->base);
	mutex_unlock(&vm->vm_lock);
	if (ret)
		goto err_free_range;

	vm_bo = drm_gpuvm_bo_obtain(&vm->base, obj);
	if (IS_ERR(vm_bo)) {
		ret = PTR_ERR(vm_bo);
		goto err_va_remove;
	}

	mutex_lock(&vm->vm_lock);
	drm_gpuva_link(&vma->base, vm_bo);
	mutex_unlock(&vm->vm_lock);
	GEM_WARN_ON(drm_gpuvm_bo_put(vm_bo));

	return &vma->base;

err_va_remove:
	mutex_lock(&vm->vm_lock);
	drm_gpuva_remove(&vma->base);
	mutex_unlock(&vm->vm_lock);
err_free_range:
	if (vm->managed)
		drm_mm_remove_node(&vma->node);
err_free_vma:
	kfree(vma);
	return ERR_PTR(ret);
}

static const struct drm_gpuvm_ops msm_gpuvm_ops = {
	.vm_free = msm_gem_vm_free,
};

static int
run_bo_op(struct msm_gem_submit *submit, const struct msm_gem_submit_bo *bo)
{
	unsigned op = bo->flags & MSM_SUBMIT_BO_OP_MASK;

	switch (op) {
	case MSM_SUBMIT_BO_OP_MAP:
	case MSM_SUBMIT_BO_OP_MAP_NULL:
		return drm_gpuvm_sm_map(submit->vm, submit->vm, bo->iova,
					bo->range, bo->obj, bo->bo_offset);
		break;
	case MSM_SUBMIT_BO_OP_UNMAP:
		return drm_gpuvm_sm_unmap(submit->vm, submit->vm, bo->iova,
					  bo->bo_offset);
	}

	return -EINVAL;
}

static struct dma_fence *
msm_vma_job_run(struct drm_sched_job *job)
{
	struct msm_gem_submit *submit = to_msm_submit(job);

	for (unsigned i = 0; i < submit->nr_bos; i++) {
		int ret = run_bo_op(submit, &submit->bos[i]);
		if (ret) {
			to_msm_vm(submit->vm)->unusable = true;
			return ERR_PTR(ret);
		}
	}

	/* VM_BIND ops run on CPU, so we are done now: */
	msm_submit_retire(submit);

	for (int i = 0; i < submit->nr_bos; i++) {
		struct drm_gem_object *obj = submit->bos[i].obj;

		if (!obj)
			continue;

		msm_gem_lock(obj);
		msm_gem_unpin_locked(obj);
		msm_gem_unlock(obj);
	}

	/* VM_BIND ops are synchronous, so no fence to wait on: */
	return NULL;
}

static void
msm_vma_job_free(struct drm_sched_job *job)
{
	struct msm_gem_submit *submit = to_msm_submit(job);

	drm_sched_job_cleanup(job);
	msm_gem_submit_put(submit);
}

static const struct drm_sched_backend_ops msm_vm_bind_ops = {
	.run_job = msm_vma_job_run,
	.free_job = msm_vma_job_free
};

/**
 * msm_gem_vm_create() - Create and initialize a &msm_gem_vm
 * @drm: the drm device
 * @mmu: the backing MMU objects handling mapping/unmapping
 * @name: the name of the VM
 * @va_start: the start offset of the GPU VA space
 * @va_size: the size of the GPU VA space
 * @managed: is it a kernel managed VM?
 *
 * In a kernel managed VM, the kernel handles address allocation, and only
 * synchronous operations are supported.  In a user managed VM, userspace
 * handles virtual address allocation, and both async and sync operations
 * are supported.
 */
struct drm_gpuvm *
msm_gem_vm_create(struct drm_device *drm, struct msm_mmu *mmu, const char *name,
		  u64 va_start, u64 va_size, bool managed)
{
	struct msm_gem_vm *vm;
	struct drm_gem_object *dummy_gem;
	int ret = 0;

	if (IS_ERR(mmu))
		return ERR_CAST(mmu);

	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return ERR_PTR(-ENOMEM);

	dummy_gem = drm_gpuvm_resv_object_alloc(drm);
	if (!dummy_gem) {
		ret = -ENOMEM;
		goto err_free_vm;
	}

	if (!managed) {
		ret = drm_sched_init(&vm->sched, &msm_vm_bind_ops, NULL, 1, 1, 0,
				     MAX_SCHEDULE_TIMEOUT, NULL, NULL,
				     "msm-vm-bind", drm->dev);
		if (ret)
			goto err_free_dummy;
	}

	drm_gpuvm_init(&vm->base, name, 0, drm, dummy_gem,
		       va_start, va_size, 0, 0, &msm_gpuvm_ops);
	drm_gem_object_put(dummy_gem);

	spin_lock_init(&vm->mm_lock);
	mutex_init(&vm->vm_lock);

	vm->mmu = mmu;
	vm->managed = managed;

	drm_mm_init(&vm->mm, va_start, va_size);

	return &vm->base;

err_free_dummy:
	drm_gem_object_put(dummy_gem);

err_free_vm:
	kfree(vm);
	return ERR_PTR(ret);

}

/**
 * msm_gem_vm_close() - Close a VM
 * @gpuvm: The VM to close
 *
 * Called when the drm device file is closed, to tear down VM related resources
 * (which will drop refcounts to GEM objects that were still mapped into the
 * VM at the time).
 */
void
msm_gem_vm_close(struct drm_gpuvm *gpuvm)
{
	struct msm_gem_vm *vm = to_msm_vm(gpuvm);

	/*
	 * For kernel managed VMs, the VMAs are torn down when the handle is
	 * closed, so nothing more to do.
	 */
	if (vm->managed)
		return;

	/* Kill the scheduler now, so we aren't racing with it for cleanup: */
	drm_sched_stop(&vm->sched, NULL);
	drm_sched_fini(&vm->sched);
}
