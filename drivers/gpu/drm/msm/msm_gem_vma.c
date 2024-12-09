// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2016 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 */

#include "msm_drv.h"
#include "msm_gem.h"
#include "msm_mmu.h"

#define vm_dbg(fmt, ...) pr_debug("%s:%d: "fmt"\n", __func__, __LINE__, ##__VA_ARGS__)

static void
msm_gem_vm_free(struct drm_gpuvm *gpuvm)
{
	struct msm_gem_vm *vm = container_of(gpuvm, struct msm_gem_vm, base);
	struct msm_drm_private *priv = gpuvm->drm->dev_private;

	mutex_lock(&priv->vm_lock);
	list_del(&vm->node);
	mutex_unlock(&priv->vm_lock);

	drm_mm_takedown(&vm->mm);
	if (vm->mmu)
		vm->mmu->funcs->destroy(vm->mmu);
	put_pid(vm->pid);
	kfree(vm);
}

static void
msm_gem_vma_unmap_range(struct drm_gpuva *vma, uint64_t unmap_start, uint64_t unmap_range)
{
	struct msm_gem_vm *vm = to_msm_vm(vma->vm);

	vm_dbg("%p:%p: %016llx %016llx", vma->vm, vma, unmap_start, unmap_start + unmap_range);

	if (vma->gem.obj)
		msm_gem_assert_locked(vma->gem.obj);

	vm->mmu->funcs->unmap(vm->mmu, unmap_start, unmap_range);
}

/* Actually unmap memory for the vma */
void msm_gem_vma_unmap(struct drm_gpuva *vma)
{
	struct msm_gem_vma *msm_vma = to_msm_vma(vma);

	/* Don't do anything if the memory isn't mapped */
	if (!msm_vma->mapped)
		return;

	msm_gem_vma_unmap_range(vma, vma->va.addr, vma->va.range);

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

	vm_dbg("%p: %016llx %016llx", vma, vma->va.addr, vma->va.range);

	if (vma->gem.obj)
		msm_gem_assert_locked(vma->gem.obj);

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

	if (vma->va.addr && vm->managed) {
		spin_lock(&vm->mm_lock);
		drm_mm_remove_node(&msm_vma->node);
		spin_unlock(&vm->mm_lock);
	}

	mutex_lock(&vm->vm_lock);
	drm_gpuva_remove(vma);
	drm_gpuva_unlink(vma);
	mutex_unlock(&vm->vm_lock);

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
		BUG_ON(!obj);  /* NULL mappings not valid for kernel managed VM */
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

	if (obj)
		GEM_WARN_ON((range_end - range_start) > obj->size);

	drm_gpuva_init(&vma->base, range_start, range_end - range_start, obj, offset);
	vma->mapped = false;

	mutex_lock(&vm->vm_lock);
	ret = drm_gpuva_insert(&vm->base, &vma->base);
	mutex_unlock(&vm->vm_lock);
	if (ret)
		goto err_free_range;

	if (!obj)
		return &vma->base;

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

static int
msm_gem_vm_bo_validate(struct drm_gpuvm_bo *vm_bo, struct drm_exec *exec)
{
	// TODO
	pr_err("%s:%d\n", __func__, __LINE__);
	return 0;
}

struct op_priv {
	unsigned flags;
	struct drm_gpuvm *vm;
};

static struct drm_gpuva *
vma_from_op(struct drm_gpuvm *vm, struct drm_gpuva_op_map *op)
{
	return msm_gem_vma_new(vm, op->gem.obj, op->gem.offset, op->va.addr,
			       op->va.addr + op->va.range);
}

/*
 * In a few places, we have to deal with map/unmap of potentially NULL (PRR)
 * mappings.  The cond_lock()/cond_unlock() helpers simplify that.
 */

static void
cond_lock(struct drm_gem_object *obj)
{
	if (!obj)
		return;

	/*
	 * Hold a ref while we have the obj locked, so drm_gpuvm doesn't
	 * manage to drop the last ref to the obj while it is locked:
	 */
	drm_gem_object_get(obj);
	msm_gem_lock(obj);
}

static void
cond_unlock(struct drm_gem_object *obj)
{
	if (!obj)
		return;

	msm_gem_unlock(obj);
	/* Drop the ref obtained in cond_lock(): */
	drm_gem_object_put(obj);
}

static int
msm_gem_vm_sm_step_map(struct drm_gpuva_op *op, void *priv)
{
	struct drm_gem_object *obj = op->map.gem.obj;
	struct drm_gpuvm *vm = ((struct op_priv *)priv)->vm;
	struct drm_gpuva *vma;
	struct sg_table *sgt;
	unsigned prot;
	int ret;

	cond_lock(obj);
	vma = vma_from_op(vm, &op->map);

	vm_dbg("%p:%p: %016llx %016llx", vma->vm, vma, vma->va.addr, vma->va.range);

	if (obj) {
		sgt = to_msm_bo(obj)->sgt;
		prot = msm_gem_prot(obj);
	} else {
		sgt = NULL;
		prot = IOMMU_READ | IOMMU_WRITE;
	}

	if (WARN_ON(IS_ERR(vma))) {
		ret = PTR_ERR(vma);
		goto out_unlock;
	}

	vma->flags = ((struct op_priv *)priv)->flags;

	ret = msm_gem_vma_map(vma, prot, sgt);

out_unlock:
	cond_unlock(obj);

	return ret;
}

static int
msm_gem_vm_sm_step_remap(struct drm_gpuva_op *op, void *priv)
{
	struct drm_gpuvm *vm = ((struct op_priv *)priv)->vm;
	struct drm_gpuva *orig_vma = op->remap.unmap->va;
	struct drm_gpuva *prev_vma = NULL, *next_vma = NULL;
	struct drm_gem_object *obj = orig_vma->gem.obj;
	uint64_t unmap_start, unmap_range;
	unsigned flags;
	int ret = 0;

	vm_dbg("orig_vma: %p:%p: %016llx %016llx", vm, orig_vma, orig_vma->va.addr, orig_vma->va.range);

	drm_gpuva_op_remap_to_unmap_range(&op->remap, &unmap_start, &unmap_range);

	cond_lock(obj);
	msm_gem_vma_unmap_range(op->remap.unmap->va, unmap_start, unmap_range);

	/*
	 * Part of this GEM obj is still mapped, but we're going to kill the
	 * existing VMA and replace it with one or two new ones (ie. two if)
	 * the unmapped range is in the middle of the existing (unmap) VMA.
	 * So just set the state to unmapped:
	 */
	to_msm_vma(orig_vma)->mapped = false;

	/*
	 * The prev_vma and/or next_vma are replacing the unmapped vma, and
	 * therefore should preserve it's flags:
	 */
	flags = orig_vma->flags;

	msm_gem_vma_close(orig_vma);

	if (op->remap.prev) {
		prev_vma = vma_from_op(vm, op->remap.prev);
		if (WARN_ON(IS_ERR(prev_vma))) {
			ret = PTR_ERR(prev_vma);
			goto out_unlock;
		}
		vm_dbg("prev_vma: %p:%p: %016llx %016llx", vm, prev_vma, prev_vma->va.addr, prev_vma->va.range);
		to_msm_vma(prev_vma)->mapped = true;
		prev_vma->flags = flags;
	}

	if (op->remap.next) {
		next_vma = vma_from_op(vm, op->remap.next);
		if (WARN_ON(IS_ERR(next_vma))) {
			ret = PTR_ERR(next_vma);
			goto out_unlock;
		}
		vm_dbg("next_vma: %p:%p: %016llx %016llx", vm, next_vma, next_vma->va.addr, next_vma->va.range);
		to_msm_vma(next_vma)->mapped = true;
		next_vma->flags = flags;
	}

out_unlock:
	cond_unlock(obj);

	return ret;
}

static int
msm_gem_vm_sm_step_unmap(struct drm_gpuva_op *op, void *priv)
{
	struct drm_gpuva *vma = op->unmap.va;
	struct drm_gem_object *obj = vma->gem.obj;

	vm_dbg("%p:%p: %016llx %016llx", vma->vm, vma, vma->va.addr, vma->va.range);

	cond_lock(obj);
	msm_gem_vma_unmap(vma);
	msm_gem_vma_close(vma);
	cond_unlock(obj);

	return 0;
}

static const struct drm_gpuvm_ops msm_gpuvm_ops = {
	.vm_free = msm_gem_vm_free,
	.vm_bo_validate = msm_gem_vm_bo_validate,
	.sm_step_map = msm_gem_vm_sm_step_map,
	.sm_step_remap = msm_gem_vm_sm_step_remap,
	.sm_step_unmap = msm_gem_vm_sm_step_unmap,
};

static int
run_and_free_steps(struct drm_gpuvm *vm, struct drm_gpuva_ops *ops, unsigned flags)
{
	struct op_priv priv = {
		.vm = vm,
		.flags = flags,
	};
	struct drm_gpuva_op *op;
	int ret = 0;

	if (IS_ERR(ops))
		return PTR_ERR(ops);

	drm_gpuva_for_each_op (op, ops) {

		switch (op->op) {
		case DRM_GPUVA_OP_MAP:
			ret = msm_gem_vm_sm_step_map(op, &priv);
			break;
		case DRM_GPUVA_OP_REMAP:
			ret = msm_gem_vm_sm_step_remap(op, &priv);
			break;
		case DRM_GPUVA_OP_UNMAP:
			ret = msm_gem_vm_sm_step_unmap(op, &priv);
			break;
		default:
			ret = -EINVAL;
		}

		if (ret)
			break;
	}

	drm_gpuva_ops_free(vm, ops);

	return ret;
}

static int
run_bo_op(struct msm_gem_submit *submit, const struct msm_gem_submit_bo *bo)
{
	struct msm_gem_vm *vm = to_msm_vm(submit->vm);
	struct drm_gpuva_ops *ops;
	unsigned flags = 0;

	mutex_lock(&vm->vm_lock);
	switch (bo->flags & MSM_SUBMIT_BO_OP_MASK) {
	case MSM_SUBMIT_BO_OP_MAP:
		if (bo->flags & MSM_SUBMIT_BO_DUMP)
			flags |= MSM_VMA_DUMP;
		fallthrough;
	case MSM_SUBMIT_BO_OP_MAP_NULL:
		vm_dbg("MAP: %p: %016llx %016llx", vm, bo->iova, bo->range);
		ops = drm_gpuvm_sm_map_ops_create(submit->vm, bo->iova,
						  bo->range, bo->obj,
						  bo->bo_offset);
		break;
	case MSM_SUBMIT_BO_OP_UNMAP:
		vm_dbg("UNMAP: %p: %016llx %016llx", vm, bo->iova, bo->range);
		ops = drm_gpuvm_sm_unmap_ops_create(submit->vm, bo->iova,
						    bo->bo_offset);
		break;
	default:
		ops = ERR_PTR(-EINVAL);
		break;
	}
	mutex_unlock(&vm->vm_lock);

	return run_and_free_steps(submit->vm, ops, flags);
}

static struct dma_fence *
msm_vma_job_run(struct drm_sched_job *job)
{
	struct msm_gem_submit *submit = to_msm_submit(job);
	struct msm_gem_vm *vm = to_msm_vm(submit->vm);

	mutex_lock(&vm->op_lock);

	for (unsigned i = 0; i < submit->nr_bos; i++) {
		int ret = run_bo_op(submit, &submit->bos[i]);
		if (ret) {
			to_msm_vm(submit->vm)->unusable = true;
			mutex_unlock(&vm->op_lock);
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

	mutex_unlock(&vm->op_lock);

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
	struct msm_drm_private *priv = drm->dev_private;
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

	/* For userspace pgtables, generate a VM name based on comm and PID nr: */
	if (!name) {
		char tmpname[TASK_COMM_LEN];
		struct pid *pid = get_pid(task_tgid(current));

		get_task_comm(tmpname, current);
		rcu_read_lock();
		snprintf(vm->name, sizeof(name), "%s[%d]", tmpname, pid_nr(pid));
		rcu_read_unlock();

		name = vm->name;
	}

	drm_gpuvm_init(&vm->base, name, 0, drm, dummy_gem,
		       va_start, va_size, 0, 0, &msm_gpuvm_ops);
	drm_gem_object_put(dummy_gem);

	spin_lock_init(&vm->mm_lock);
	mutex_init(&vm->vm_lock);
	mutex_init(&vm->op_lock);

	vm->mmu = mmu;
	vm->managed = managed;

	drm_mm_init(&vm->mm, va_start, va_size);

	mutex_lock(&priv->vm_lock);
	list_add_tail(&vm->node, &priv->vms);
	mutex_unlock(&priv->vm_lock);

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
	struct drm_gpuva_ops *ops;

	/*
	 * For kernel managed VMs, the VMAs are torn down when the handle is
	 * closed, so nothing more to do.
	 */
	if (vm->managed)
		return;

	/* Kill the scheduler now, so we aren't racing with it for cleanup: */
	drm_sched_stop(&vm->sched, NULL);
	drm_sched_fini(&vm->sched);

	/* Serialize against vm scheduler thread: */
	mutex_lock(&vm->op_lock);

	/*
	 * To avoid nested locking problems, while still holding the lock
	 * during the necessary vm traversal, generate a list of unmap ops:
	 */
	mutex_lock(&vm->vm_lock);
	ops = drm_gpuvm_sm_unmap_ops_create(gpuvm, gpuvm->mm_start, gpuvm->mm_range);
	mutex_unlock(&vm->vm_lock);

	run_and_free_steps(gpuvm, ops, 0);

	mutex_unlock(&vm->op_lock);
}
