/**
 * Planner Visualizer - 处理 gRPC Planner 的可视化
 * 功能：
 * 1. 显示曲面网格
 * 2. 显示游标最近点
 * 3. 显示轨迹和法向量
 */

class PlannerVisualizer {
    constructor(scene, ros) {
        this.scene = scene;
        this.ros = ros;

        // 可视化对象
        this.surfaceMesh = null;
        this.closestPointMarker = null;
        this.trajectoryLines = [];
        this.normalArrows = [];

        // 订阅器
        this.meshSubscriber = null;
        this.closestPointPosSubscriber = null;
        this.closestPointUVSubscriber = null;
        this.trajectorySubscriber = null;

        // 发布器
        this.convertMeshPublisher = null;
        this.cursorPublisher = null;
        this.planTrajectoryPublisher = null;

        // 状态
        this.surfaceGenerated = false;
        this.currentUV = { u: 0, v: 0 };

        // 游标模式
        this.cursorMode = false;  // 是否启用游标模式
        this.onPointSelected = null;  // 点选回调函数

        this.init();
    }

    init() {
        console.log('PlannerVisualizer 初始化...');
        this.setupPublishers();
        this.setupSubscribers();
    }

    setupPublishers() {
        // 发布网格转换触发信号
        this.convertMeshPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/convert_mesh_trigger',
            messageType: 'std_msgs/Bool'
        });

        // 发布游标位置
        this.cursorPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/cursor_position',
            messageType: 'geometry_msgs/PointStamped'
        });

        // 发布轨迹规划请求（使用自定义消息，这里简化为 PoseStamped）
        // 实际应该定义专门的消息类型
        this.planTrajectoryPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/plan_trajectory_request',
            messageType: 'geometry_msgs/PoseStamped'
        });

        console.log('✓ Planner 发布器设置完成');
    }

    setupSubscribers() {
        // 订阅网格数据
        this.meshSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/mesh',
            messageType: 'visualization_msgs/MarkerArray'
        });

        this.meshSubscriber.subscribe((msg) => {
            this.handleMeshMessage(msg);
        });

        // 订阅最近点位置
        this.closestPointPosSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/closest_point_position',
            messageType: 'geometry_msgs/PointStamped'
        });

        this.closestPointPosSubscriber.subscribe((msg) => {
            this.handleClosestPointPosition(msg);
        });

        // 订阅最近点 UV 参数
        this.closestPointUVSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/closest_point_uv',
            messageType: 'geometry_msgs/Vector3'
        });

        this.closestPointUVSubscriber.subscribe((msg) => {
            this.handleClosestPointUV(msg);
        });

        // 订阅轨迹数据
        this.trajectorySubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/trajectory',
            messageType: 'visualization_msgs/MarkerArray'
        });

        this.trajectorySubscriber.subscribe((msg) => {
            this.handleTrajectoryMessage(msg);
        });

        console.log('✓ Planner 订阅器设置完成');
    }

    /**
     * 触发网格转换
     */
    triggerConvertMesh() {
        const msg = new ROSLIB.Message({
            data: true
        });

        this.convertMeshPublisher.publish(msg);
        console.log('✓ 发送网格转换触发信号');
    }

    /**
     * 处理网格消息
     */
    handleMeshMessage(msg) {
        console.log('收到网格数据:', msg.markers.length, '个 marker');

        // 清除旧的网格
        if (this.surfaceMesh) {
            this.scene.remove(this.surfaceMesh);
            if (this.surfaceMesh.geometry) this.surfaceMesh.geometry.dispose();
            if (this.surfaceMesh.material) this.surfaceMesh.material.dispose();
        }

        // 解析并创建网格
        for (const marker of msg.markers) {
            if (marker.type === 11) {  // TRIANGLE_LIST
                this.createSurfaceMesh(marker);
                this.surfaceGenerated = true;
            }
        }
    }

    /**
     * 创建曲面网格
     */
    createSurfaceMesh(marker) {
        const geometry = new THREE.BufferGeometry();
        const vertices = [];

        // 提取顶点
        for (const point of marker.points) {
            vertices.push(point.x, point.y, point.z);
        }

        geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
        geometry.computeVertexNormals();

        // 创建材质
        const material = new THREE.MeshPhongMaterial({
            color: new THREE.Color(marker.color.r, marker.color.g, marker.color.b),
            transparent: true,
            opacity: marker.color.a,
            side: THREE.DoubleSide,
            flatShading: false
        });

        this.surfaceMesh = new THREE.Mesh(geometry, material);
        this.scene.add(this.surfaceMesh);

        console.log('✓ 曲面网格已创建:', vertices.length / 3, '个顶点');
    }

    /**
     * 发送游标位置
     */
    sendCursorPosition(x, y, z) {
        if (!this.cursorMode) return;

        const msg = new ROSLIB.Message({
            header: {
                frame_id: 'web_frame',
                stamp: { secs: 0, nsecs: 0 }
            },
            point: { x: x, y: y, z: z }
        });

        this.cursorPublisher.publish(msg);
    }

    /**
     * 处理最近点位置
     */
    handleClosestPointPosition(msg) {
        const pos = msg.point;

        // 更新或创建最近点标记
        if (!this.closestPointMarker) {
            const geometry = new THREE.SphereGeometry(0.05, 16, 16);
            const material = new THREE.MeshBasicMaterial({
                color: 0xff00ff,
                transparent: true,
                opacity: 0.8
            });
            this.closestPointMarker = new THREE.Mesh(geometry, material);
            this.scene.add(this.closestPointMarker);
        }

        this.closestPointMarker.position.set(pos.x, pos.y, pos.z);
        this.closestPointMarker.visible = this.cursorMode;
    }

    /**
     * 处理最近点 UV 参数
     */
    handleClosestPointUV(msg) {
        this.currentUV.u = msg.x;
        this.currentUV.v = msg.y;
    }

    /**
     * 启用/禁用游标模式
     */
    setCursorMode(enabled, onPointSelected = null) {
        this.cursorMode = enabled;
        this.onPointSelected = onPointSelected;

        if (this.closestPointMarker) {
            this.closestPointMarker.visible = enabled;
        }

        console.log('游标模式:', enabled ? '启用' : '禁用');
    }

    /**
     * 确认选择当前点
     */
    confirmPointSelection() {
        if (this.onPointSelected && this.closestPointMarker) {
            const pos = this.closestPointMarker.position;
            const uv = { ...this.currentUV };
            this.onPointSelected(pos, uv);
        }
    }

    /**
     * 发送轨迹规划请求
     */
    sendPlanTrajectoryRequest(startUV, goalUV, numSamples = 5000, timeout = 20.0) {
        // 由于 ROS 没有直接支持自定义消息，我们使用 PoseStamped 来传递参数
        // position.x = start_u, position.y = start_v
        // orientation.x = goal_u, orientation.y = goal_v
        const msg = new ROSLIB.Message({
            header: {
                frame_id: 'web_frame',
                stamp: { secs: 0, nsecs: 0 }
            },
            pose: {
                position: {
                    x: startUV.u,
                    y: startUV.v,
                    z: numSamples
                },
                orientation: {
                    x: goalUV.u,
                    y: goalUV.v,
                    z: timeout,
                    w: 1.0
                }
            }
        });

        this.planTrajectoryPublisher.publish(msg);
        console.log('✓ 发送轨迹规划请求:', startUV, '->', goalUV);
    }

    /**
     * 处理轨迹消息
     */
    handleTrajectoryMessage(msg) {
        console.log('收到轨迹数据:', msg.markers.length, '个 marker');

        // 清除旧的轨迹
        this.clearTrajectory();

        // 解析并创建轨迹可视化
        for (const marker of msg.markers) {
            if (marker.ns === 'uav_trajectory') {
                this.createTrajectoryLine(marker, 0xff0000);  // 红色 UAV 轨迹
            } else if (marker.ns === 'surface_trajectory') {
                this.createTrajectoryLine(marker, 0x00ff00);  // 绿色曲面轨迹
            } else if (marker.ns === 'surface_normals') {
                this.createNormalArrow(marker);
            }
        }

        console.log('✓ 轨迹可视化已创建');
    }

    /**
     * 创建轨迹线
     */
    createTrajectoryLine(marker, color) {
        const points = [];

        for (const point of marker.points) {
            points.push(new THREE.Vector3(point.x, point.y, point.z));
        }

        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({
            color: color,
            linewidth: 2
        });

        const line = new THREE.Line(geometry, material);
        this.scene.add(line);
        this.trajectoryLines.push(line);
    }

    /**
     * 创建法向量箭头
     */
    createNormalArrow(marker) {
        if (marker.points.length < 2) return;

        const start = marker.points[0];
        const end = marker.points[1];

        const dir = new THREE.Vector3(
            end.x - start.x,
            end.y - start.y,
            end.z - start.z
        );

        const length = dir.length();
        dir.normalize();

        const origin = new THREE.Vector3(start.x, start.y, start.z);
        const arrow = new THREE.ArrowHelper(
            dir,
            origin,
            length,
            0x0000ff,  // 蓝色
            length * 0.2,  // 箭头头部长度
            length * 0.15   // 箭头头部宽度
        );

        this.scene.add(arrow);
        this.normalArrows.push(arrow);
    }

    /**
     * 清除轨迹
     */
    clearTrajectory() {
        // 清除轨迹线
        for (const line of this.trajectoryLines) {
            this.scene.remove(line);
            if (line.geometry) line.geometry.dispose();
            if (line.material) line.material.dispose();
        }
        this.trajectoryLines = [];

        // 清除法向量箭头
        for (const arrow of this.normalArrows) {
            this.scene.remove(arrow);
            if (arrow.line && arrow.line.geometry) arrow.line.geometry.dispose();
            if (arrow.line && arrow.line.material) arrow.line.material.dispose();
            if (arrow.cone && arrow.cone.geometry) arrow.cone.geometry.dispose();
            if (arrow.cone && arrow.cone.material) arrow.cone.material.dispose();
        }
        this.normalArrows = [];
    }

    /**
     * 清除所有可视化
     */
    clearAll() {
        // 清除网格
        if (this.surfaceMesh) {
            this.scene.remove(this.surfaceMesh);
            if (this.surfaceMesh.geometry) this.surfaceMesh.geometry.dispose();
            if (this.surfaceMesh.material) this.surfaceMesh.material.dispose();
            this.surfaceMesh = null;
        }

        // 清除最近点标记
        if (this.closestPointMarker) {
            this.scene.remove(this.closestPointMarker);
            if (this.closestPointMarker.geometry) this.closestPointMarker.geometry.dispose();
            if (this.closestPointMarker.material) this.closestPointMarker.material.dispose();
            this.closestPointMarker = null;
        }

        // 清除轨迹
        this.clearTrajectory();

        this.surfaceGenerated = false;
        console.log('✓ 已清除所有 Planner 可视化');
    }

    /**
     * 显示/隐藏曲面网格
     */
    setSurfaceVisible(visible) {
        if (this.surfaceMesh) {
            this.surfaceMesh.visible = visible;
        }
    }

    /**
     * 显示/隐藏轨迹
     */
    setTrajectoryVisible(visible) {
        for (const line of this.trajectoryLines) {
            line.visible = visible;
        }
        for (const arrow of this.normalArrows) {
            arrow.visible = visible;
        }
    }

    /**
     * 清理资源
     */
    dispose() {
        this.clearAll();

        // 取消订阅
        if (this.meshSubscriber) this.meshSubscriber.unsubscribe();
        if (this.closestPointPosSubscriber) this.closestPointPosSubscriber.unsubscribe();
        if (this.closestPointUVSubscriber) this.closestPointUVSubscriber.unsubscribe();
        if (this.trajectorySubscriber) this.trajectorySubscriber.unsubscribe();

        console.log('PlannerVisualizer 已清理');
    }
}
