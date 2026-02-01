/**
 * ROI 选择器模块
 * 负责创建和管理 3D 裁剪框
 * 支持6个方向独立缩放
 */

class ROISelector {
    constructor(scene, camera, renderer, transformControls) {
        this.scene = scene;
        this.camera = camera;
        this.renderer = renderer;
        this.transformControls = transformControls;

        this.roiBox = null;
        this.roiHelper = null;
        this.isActive = false;

        // 默认裁剪框参数
        this.defaultSize = { x: 2, y: 2, z: 2 };
        this.defaultPosition = { x: 0, y: 0, z: 1 };

        // 当前实际尺寸
        this.currentSize = { ...this.defaultSize };

        // 缩放控制点
        this.scaleHandles = [];
        this.isScaleMode = false;
        this.isDraggingHandle = false;
        this.activeHandle = null;
        this.dragPlane = new THREE.Plane();
        this.dragStart = new THREE.Vector3();
        this.handleStartPos = new THREE.Vector3();
        this.boxStartSize = { x: 0, y: 0, z: 0 };
        this.boxStartPos = new THREE.Vector3();

        // Raycaster for handle picking
        this.raycaster = new THREE.Raycaster();
        this.raycaster.params.Points.threshold = 0.1;
        this.mouse = new THREE.Vector2();

        // 绑定事件处理函数
        this.onMouseDown = this.onMouseDown.bind(this);
        this.onMouseMove = this.onMouseMove.bind(this);
        this.onMouseUp = this.onMouseUp.bind(this);
    }

    /**
     * 创建 ROI 裁剪框
     */
    create() {
        if (this.roiBox) {
            console.warn('ROI 裁剪框已存在');
            return;
        }

        // 重置当前尺寸
        this.currentSize = { ...this.defaultSize };

        // 创建半透明盒子几何体
        const geometry = new THREE.BoxGeometry(1, 1, 1);

        // 创建半透明材质
        const material = new THREE.MeshBasicMaterial({
            color: 0x00ff00,
            transparent: true,
            opacity: 0.3,
            side: THREE.DoubleSide
        });

        this.roiBox = new THREE.Mesh(geometry, material);
        this.roiBox.position.set(
            this.defaultPosition.x,
            this.defaultPosition.y,
            this.defaultPosition.z
        );
        this.roiBox.scale.set(this.currentSize.x, this.currentSize.y, this.currentSize.z);
        this.roiBox.name = 'roi_box';

        // 创建边框辅助线
        this.updateEdgeHelper();

        // 添加到场景
        this.scene.add(this.roiBox);

        // 附加到变换控制器（默认平移模式）
        this.transformControls.attach(this.roiBox);

        this.isActive = true;
        console.log('ROI 裁剪框已创建');
    }

    /**
     * 更新边框辅助线
     */
    updateEdgeHelper() {
        // 移除旧的辅助线
        if (this.roiHelper) {
            this.roiBox.remove(this.roiHelper);
            this.roiHelper.geometry.dispose();
            this.roiHelper.material.dispose();
        }

        const edges = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
        const lineMaterial = new THREE.LineBasicMaterial({
            color: 0x00ff00,
            linewidth: 2
        });
        this.roiHelper = new THREE.LineSegments(edges, lineMaterial);
        this.roiBox.add(this.roiHelper);
    }

    /**
     * 创建6个方向的缩放控制点
     */
    createScaleHandles() {
        // 清除旧的控制点
        this.removeScaleHandles();

        const handleSize = 0.18;
        const handleGeometry = new THREE.SphereGeometry(handleSize, 16, 16);

        // 6个方向: +X, -X, +Y, -Y, +Z, -Z
        const directions = [
            { name: 'posX', dir: new THREE.Vector3(1, 0, 0), color: 0xff4444 },
            { name: 'negX', dir: new THREE.Vector3(-1, 0, 0), color: 0xff8888 },
            { name: 'posY', dir: new THREE.Vector3(0, 1, 0), color: 0x44ff44 },
            { name: 'negY', dir: new THREE.Vector3(0, -1, 0), color: 0x88ff88 },
            { name: 'posZ', dir: new THREE.Vector3(0, 0, 1), color: 0x4444ff },
            { name: 'negZ', dir: new THREE.Vector3(0, 0, -1), color: 0x8888ff }
        ];

        directions.forEach(({ name, dir, color }) => {
            const material = new THREE.MeshBasicMaterial({
                color: color,
                transparent: true,
                opacity: 0.9
            });
            const handle = new THREE.Mesh(handleGeometry, material);
            handle.name = `scale_handle_${name}`;
            handle.userData = {
                isScaleHandle: true,
                direction: dir.clone(),
                handleName: name
            };

            this.scene.add(handle);
            this.scaleHandles.push(handle);
        });

        this.updateScaleHandlePositions();
    }

    /**
     * 更新缩放控制点位置
     */
    updateScaleHandlePositions() {
        if (!this.roiBox || this.scaleHandles.length === 0) return;

        const boxPos = this.roiBox.position;
        const boxQuat = this.roiBox.quaternion;
        const size = this.currentSize;

        this.scaleHandles.forEach(handle => {
            const dir = handle.userData.direction.clone();
            // 计算控制点在box表面的位置
            const localPos = new THREE.Vector3(
                dir.x * size.x / 2,
                dir.y * size.y / 2,
                dir.z * size.z / 2
            );
            // 应用box的旋转
            localPos.applyQuaternion(boxQuat);
            // 加上box的位置
            handle.position.copy(boxPos).add(localPos);
        });
    }

    /**
     * 移除缩放控制点
     */
    removeScaleHandles() {
        this.scaleHandles.forEach(handle => {
            this.scene.remove(handle);
            handle.geometry.dispose();
            handle.material.dispose();
        });
        this.scaleHandles = [];
    }

    /**
     * 删除 ROI 裁剪框
     */
    delete() {
        if (!this.roiBox) {
            return;
        }

        // 移除事件监听
        this.disableScaleMode();

        // 移除缩放控制点
        this.removeScaleHandles();

        // 从变换控制器分离
        this.transformControls.detach();

        // 从场景移除
        this.scene.remove(this.roiBox);

        // 清理资源
        this.roiBox.geometry.dispose();
        this.roiBox.material.dispose();
        if (this.roiHelper) {
            this.roiHelper.geometry.dispose();
            this.roiHelper.material.dispose();
        }

        this.roiBox = null;
        this.roiHelper = null;
        this.isActive = false;

        console.log('ROI 裁剪框已删除');
    }

    /**
     * 设置变换模式
     */
    setTransformMode(mode) {
        if (mode === 'scale') {
            // 进入自定义缩放模式
            this.enableScaleMode();
        } else {
            // 退出缩放模式，使用TransformControls
            this.disableScaleMode();
            if (this.transformControls && this.roiBox) {
                this.transformControls.attach(this.roiBox);
                this.transformControls.setMode(mode);
            }
        }
    }

    /**
     * 启用自定义缩放模式
     */
    enableScaleMode() {
        if (!this.roiBox) return;

        this.isScaleMode = true;

        // 分离TransformControls
        this.transformControls.detach();

        // 创建并显示缩放控制点
        this.createScaleHandles();

        // 添加事件监听到canvas
        const canvas = this.renderer.domElement;
        canvas.addEventListener('pointerdown', this.onMouseDown, false);
        canvas.addEventListener('pointermove', this.onMouseMove, false);
        canvas.addEventListener('pointerup', this.onMouseUp, false);
        canvas.addEventListener('pointerleave', this.onMouseUp, false);

        console.log('进入缩放模式');
    }

    /**
     * 禁用自定义缩放模式
     */
    disableScaleMode() {
        this.isScaleMode = false;
        this.isDraggingHandle = false;
        this.activeHandle = null;

        // 移除缩放控制点
        this.removeScaleHandles();

        // 移除事件监听
        const canvas = this.renderer.domElement;
        canvas.removeEventListener('pointerdown', this.onMouseDown, false);
        canvas.removeEventListener('pointermove', this.onMouseMove, false);
        canvas.removeEventListener('pointerup', this.onMouseUp, false);
        canvas.removeEventListener('pointerleave', this.onMouseUp, false);

        // 恢复鼠标样式
        canvas.style.cursor = 'default';
    }

    /**
     * 获取鼠标/触摸位置
     */
    getPointerPosition(event) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    }

    /**
     * 鼠标按下事件
     */
    onMouseDown(event) {
        if (!this.isScaleMode || !this.roiBox) return;

        this.getPointerPosition(event);
        this.raycaster.setFromCamera(this.mouse, this.camera);

        const intersects = this.raycaster.intersectObjects(this.scaleHandles, false);

        if (intersects.length > 0) {
            event.preventDefault();
            event.stopPropagation();

            this.isDraggingHandle = true;
            this.activeHandle = intersects[0].object;

            // 记录起始状态
            this.handleStartPos.copy(this.activeHandle.position);
            this.boxStartSize = { ...this.currentSize };
            this.boxStartPos.copy(this.roiBox.position);

            // 创建拖拽平面（垂直于相机方向）
            const cameraDir = new THREE.Vector3();
            this.camera.getWorldDirection(cameraDir);
            this.dragPlane.setFromNormalAndCoplanarPoint(cameraDir, this.activeHandle.position);

            // 记录拖拽起始点
            const startPoint = new THREE.Vector3();
            this.raycaster.ray.intersectPlane(this.dragPlane, startPoint);
            this.dragStart.copy(startPoint);

            // 禁用OrbitControls
            if (window.app && window.app.orbitControls) {
                window.app.orbitControls.enabled = false;
            }

            // 高亮当前控制点
            this.activeHandle.material.opacity = 1.0;
            this.activeHandle.scale.set(1.3, 1.3, 1.3);

            this.renderer.domElement.style.cursor = 'grabbing';
        }
    }

    /**
     * 鼠标移动事件
     */
    onMouseMove(event) {
        if (!this.isScaleMode || !this.roiBox) return;

        this.getPointerPosition(event);
        this.raycaster.setFromCamera(this.mouse, this.camera);

        if (this.isDraggingHandle && this.activeHandle) {
            event.preventDefault();

            // 计算拖拽位移
            const dragCurrent = new THREE.Vector3();
            if (!this.raycaster.ray.intersectPlane(this.dragPlane, dragCurrent)) {
                return;
            }

            const delta = dragCurrent.clone().sub(this.dragStart);

            // 获取控制点的方向（在世界坐标系中）
            const localDir = this.activeHandle.userData.direction.clone();
            const worldDir = localDir.clone().applyQuaternion(this.roiBox.quaternion);

            // 计算沿着方向的位移量
            // displacement > 0 表示沿着worldDir方向移动（向外）
            // displacement < 0 表示沿着worldDir反方向移动（向内）
            const displacement = delta.dot(worldDir);

            // 根据方向更新尺寸
            const handleName = this.activeHandle.userData.handleName;
            const axis = handleName.includes('X') ? 'x' : (handleName.includes('Y') ? 'y' : 'z');

            // 无论正负方向的球体，displacement都表示沿着该球体指向外的方向移动的距离
            // 所以直接用displacement作为尺寸变化量
            const minSize = 0.3;
            let newSize = this.boxStartSize[axis] + displacement;
            newSize = Math.max(minSize, newSize);

            // 计算尺寸变化
            const sizeDelta = newSize - this.boxStartSize[axis];

            // 更新当前尺寸
            this.currentSize[axis] = newSize;

            // 更新box的scale
            this.roiBox.scale.set(this.currentSize.x, this.currentSize.y, this.currentSize.z);

            // 计算位置偏移（保持对面不动）
            // 球体向外移动sizeDelta时，box中心需要向该球体方向移动sizeDelta/2
            const posOffset = worldDir.clone().multiplyScalar(sizeDelta / 2);
            this.roiBox.position.copy(this.boxStartPos).add(posOffset);

            // 更新所有控制点位置
            this.updateScaleHandlePositions();

        } else {
            // 悬停高亮
            const intersects = this.raycaster.intersectObjects(this.scaleHandles, false);

            // 重置所有控制点样式
            this.scaleHandles.forEach(handle => {
                handle.material.opacity = 0.9;
            });

            if (intersects.length > 0) {
                intersects[0].object.material.opacity = 1.0;
                this.renderer.domElement.style.cursor = 'grab';
            } else {
                this.renderer.domElement.style.cursor = 'default';
            }
        }
    }

    /**
     * 鼠标释放事件
     */
    onMouseUp(event) {
        if (this.isDraggingHandle) {
            this.isDraggingHandle = false;

            if (this.activeHandle) {
                this.activeHandle.material.opacity = 0.9;
                this.activeHandle.scale.set(1, 1, 1);
            }
            this.activeHandle = null;

            // 恢复OrbitControls
            if (window.app && window.app.orbitControls) {
                window.app.orbitControls.enabled = true;
            }

            this.renderer.domElement.style.cursor = 'default';
        }
    }

    /**
     * 获取 ROI 参数
     */
    getParameters() {
        if (!this.roiBox) {
            return null;
        }

        const position = this.roiBox.position;
        const quaternion = this.roiBox.quaternion;

        return {
            center: {
                x: position.x,
                y: position.y,
                z: position.z
            },
            size: { ...this.currentSize },
            orientation: {
                x: quaternion.x,
                y: quaternion.y,
                z: quaternion.z,
                w: quaternion.w
            }
        };
    }

    /**
     * 检查是否激活
     */
    isROIActive() {
        return this.isActive;
    }
}
