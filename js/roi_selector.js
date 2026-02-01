/**
 * ROI 选择器模块
 * 负责创建和管理 3D 裁剪框
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
    }

    /**
     * 创建 ROI 裁剪框
     */
    create() {
        if (this.roiBox) {
            console.warn('ROI 裁剪框已存在');
            return;
        }

        // 创建半透明盒子几何体
        const geometry = new THREE.BoxGeometry(
            this.defaultSize.x,
            this.defaultSize.y,
            this.defaultSize.z
        );

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
        this.roiBox.name = 'roi_box';

        // 创建边框辅助线
        const edges = new THREE.EdgesGeometry(geometry);
        const lineMaterial = new THREE.LineBasicMaterial({
            color: 0x00ff00,
            linewidth: 2
        });
        this.roiHelper = new THREE.LineSegments(edges, lineMaterial);
        this.roiBox.add(this.roiHelper);

        // 添加到场景
        this.scene.add(this.roiBox);

        // 附加到变换控制器
        this.transformControls.attach(this.roiBox);

        this.isActive = true;
        console.log('ROI 裁剪框已创建');
    }

    /**
     * 删除 ROI 裁剪框
     */
    delete() {
        if (!this.roiBox) {
            return;
        }

        // 从变换控制器分离
        this.transformControls.detach();

        // 从场景移除
        this.scene.remove(this.roiBox);

        // 清理资源
        this.roiBox.geometry.dispose();
        this.roiBox.material.dispose();
        this.roiHelper.geometry.dispose();
        this.roiHelper.material.dispose();

        this.roiBox = null;
        this.roiHelper = null;
        this.isActive = false;

        console.log('ROI 裁剪框已删除');
    }

    /**
     * 设置变换模式
     */
    setTransformMode(mode) {
        if (this.transformControls) {
            this.transformControls.setMode(mode);
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
        const scale = this.roiBox.scale;
        const quaternion = this.roiBox.quaternion;

        // 计算实际尺寸（考虑缩放）
        const size = {
            x: this.defaultSize.x * scale.x,
            y: this.defaultSize.y * scale.y,
            z: this.defaultSize.z * scale.z
        };

        return {
            center: {
                x: position.x,
                y: position.y,
                z: position.z
            },
            size: size,
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
