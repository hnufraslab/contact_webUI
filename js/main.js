/**
 * 主应用程序
 * ROS 点云裁剪与目标下发工具
 */

class ROSPointCloudApp {
    constructor() {
        // ROS 连接
        this.ros = null;
        this.isConnected = false;

        // Three.js 组件
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.orbitControls = null;
        this.transformControls = null;

        // 管理器
        this.pointCloudManager = null;
        this.roiSelector = null;

        // 订阅器
        this.poseSubscriber = null;

        // 发布器
        this.roiPublisher = null;
        this.goalPointsPublisher = null;
        this.convertMeshTriggerPublisher = null;
        this.accumulationFramesPublisher = null;

        // 状态
        this.surfaceFitted = false;  // 曲面是否已拟合
        this.meshGroup = null;  // 网格可视化组
        this.dronePosition = { x: 0, y: 0, z: 0 };
        this.droneOrientation = { x: 0, y: 0, z: 0, w: 1 };
        this.droneMesh = null;

        // 目标点
        this.startPoint = null;
        this.endPoint = null;
        this.startMarker = null;
        this.endMarker = null;

        // 点选模式
        this.pickingMode = null; // 'start' or 'end'
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();

        // 移动设备检测
        this.isMobile = this.detectMobile();
        this.isAndroid = /Android/i.test(navigator.userAgent);
        this.androidVersion = this.getAndroidVersion();

        // UI 元素
        this.initUIElements();

        // 初始化移动端菜单
        this.initMobileMenu();

        // 初始化
        this.init();
    }

    /**
     * 检测是否为移动设备
     */
    detectMobile() {
        return /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent) ||
               (window.innerWidth <= 768);
    }

    /**
     * 获取安卓版本号
     */
    getAndroidVersion() {
        const match = navigator.userAgent.match(/Android\s([0-9.]*)/);
        return match ? parseFloat(match[1]) : 0;
    }

    /**
     * 初始化移动端菜单
     */
    initMobileMenu() {
        const menuToggle = document.getElementById('mobile-menu-toggle');
        const closeBtn = document.getElementById('mobile-close-btn');
        const sidebar = document.getElementById('sidebar');

        if (menuToggle && sidebar) {
            // 点击绿色按钮切换菜单（打开/关闭）
            const toggleMenu = (e) => {
                e.preventDefault();
                e.stopPropagation();
                sidebar.classList.toggle('mobile-open');
            };
            menuToggle.addEventListener('click', toggleMenu);
            menuToggle.addEventListener('touchend', toggleMenu);
        }

        if (closeBtn && sidebar) {
            // 关闭按钮
            const closeMenu = (e) => {
                e.preventDefault();
                e.stopPropagation();
                sidebar.classList.remove('mobile-open');
            };
            closeBtn.addEventListener('click', closeMenu);
            closeBtn.addEventListener('touchend', closeMenu);
        }
    }

    /**
     * 初始化 UI 元素引用
     */
    initUIElements() {
        // 连接相关
        this.connectBtn = document.getElementById('connect-btn');
        this.rosUrlInput = document.getElementById('ros-url');
        this.connectionStatus = document.getElementById('connection-status');
        this.connectionText = document.getElementById('connection-text');

        // 设置默认 ROS URL 为当前服务器地址
        if (this.rosUrlInput && !this.rosUrlInput.value) {
            const hostname = window.location.hostname || 'localhost';
            this.rosUrlInput.value = `ws://${hostname}:9090`;
        }

        // 点云设置
        this.pointcloudTopicInput = document.getElementById('pointcloud-topic');
        this.poseTopicInput = document.getElementById('pose-topic');
        this.pointSizeSlider = document.getElementById('point-size');
        this.pointSizeValue = document.getElementById('point-size-value');
        this.cloudDensitySlider = document.getElementById('cloud-density');
        this.cloudDensityValue = document.getElementById('cloud-density-value');
        this.accumulationFramesSlider = document.getElementById('accumulation-frames');
        this.accumulationFramesValue = document.getElementById('accumulation-frames-value');
        this.showDroneCheckbox = document.getElementById('show-drone');

        // ROI 控制
        this.createRoiBtn = document.getElementById('create-roi-btn');
        this.deleteRoiBtn = document.getElementById('delete-roi-btn');
        this.roiInfo = document.getElementById('roi-info');
        this.transformModeSelect = document.getElementById('transform-mode');

        // UV 滑块控制
        this.startUSlider = document.getElementById('start-u-slider');
        this.startVSlider = document.getElementById('start-v-slider');
        this.startUValue = document.getElementById('start-u-value');
        this.startVValue = document.getElementById('start-v-value');
        this.goalUSlider = document.getElementById('goal-u-slider');
        this.goalVSlider = document.getElementById('goal-v-slider');
        this.goalUValue = document.getElementById('goal-u-value');
        this.goalVValue = document.getElementById('goal-v-value');
        this.startPointInfo = document.getElementById('start-point-info');
        this.goalPointInfo = document.getElementById('goal-point-info');
        this.numSamplesInput = document.getElementById('num-samples');
        this.planningTimeoutInput = document.getElementById('planning-timeout');

        // 执行控制
        this.executePlanningBtn = document.getElementById('execute-planning-btn');
        this.resetBtn = document.getElementById('reset-btn');

        // 裁剪控制
        this.sendCropBtn = document.getElementById('send-crop-btn');
        this.resetCropBtn = document.getElementById('reset-crop-btn');
        this.cropStatus = document.getElementById('crop-status');
        this.cropStatusText = document.getElementById('crop-status-text');

        // 曲面拟合控制
        this.generateSurfaceBtn = document.getElementById('generate-surface-btn');
        this.clearSurfaceBtn = document.getElementById('clear-surface-btn');
        this.surfaceStatusText = document.getElementById('surface-status-text');

        // 日志
        this.logContainer = document.getElementById('log-container');

        // 覆盖层
        this.pickingModeOverlay = document.getElementById('picking-mode');

        // 全屏按钮
        this.fullscreenBtn = document.getElementById('fullscreen-btn');
    }

    /**
     * 初始化应用
     */
    init() {
        this.initThreeJS();
        this.initEventListeners();
        this.animate();
        this.log('应用初始化完成', 'info');

        // 页面加载后自动连接
        setTimeout(() => {
            this.connect();
        }, 500);
    }

    /**
     * 初始化 Three.js 场景
     */
    initThreeJS() {
        const viewport = document.getElementById('viewport');
        const canvas = document.getElementById('canvas3d');

        // 创建场景
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0a0a);

        // 创建相机
        this.camera = new THREE.PerspectiveCamera(
            75,
            viewport.clientWidth / viewport.clientHeight,
            0.1,
            1000
        );
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);

        // 创建渲染器 - 简化配置以提高兼容性
        try {
            this.renderer = new THREE.WebGLRenderer({
                canvas: canvas,
                antialias: !this.isMobile
            });
        } catch (e) {
            console.error('WebGL初始化失败:', e);
            this.log('WebGL初始化失败，请检查浏览器设置', 'error');
            return;
        }

        this.renderer.setSize(viewport.clientWidth, viewport.clientHeight);

        // 限制像素比
        const pixelRatio = Math.min(window.devicePixelRatio || 1, this.isMobile ? 1.5 : 2);
        this.renderer.setPixelRatio(pixelRatio);

        // 添加光源
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 10);
        this.scene.add(directionalLight);

        // 添加网格地面
        const gridHelper = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
        this.scene.add(gridHelper);

        // 添加坐标轴
        const axesHelper = new THREE.AxesHelper(5);
        this.scene.add(axesHelper);

        // 创建轨道控制器
        this.orbitControls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.orbitControls.enableDamping = true;
        this.orbitControls.dampingFactor = 0.05;

        // 创建变换控制器
        this.transformControls = new THREE.TransformControls(this.camera, this.renderer.domElement);
        this.transformControls.addEventListener('dragging-changed', (event) => {
            this.orbitControls.enabled = !event.value;
        });
        this.transformControls.addEventListener('change', () => {
            this.updateROIInfo();
        });
        this.scene.add(this.transformControls);

        // 窗口大小调整
        window.addEventListener('resize', () => {
            this.camera.aspect = viewport.clientWidth / viewport.clientHeight;
            this.camera.updateProjectionMatrix();
            this.renderer.setSize(viewport.clientWidth, viewport.clientHeight);
        });

        this.log('Three.js 场景初始化完成', 'info');
    }

    /**
     * 初始化事件监听器
     */
    initEventListeners() {
        // 连接按钮
        this.connectBtn.addEventListener('click', () => {
            if (this.isConnected) {
                this.disconnect();
            } else {
                this.connect();
            }
        });

        // 全屏按钮
        if (this.fullscreenBtn) {
            this.fullscreenBtn.addEventListener('click', () => {
                this.toggleFullscreen();
            });

            // 监听全屏状态变化
            document.addEventListener('fullscreenchange', () => {
                this.updateFullscreenButton();
            });
            document.addEventListener('webkitfullscreenchange', () => {
                this.updateFullscreenButton();
            });
        }

        // 点云大小滑块
        this.pointSizeSlider.addEventListener('input', (e) => {
            const size = parseFloat(e.target.value);
            this.pointSizeValue.textContent = size;
            if (this.pointCloudManager) {
                this.pointCloudManager.setPointSize(size);
            }
        });

        // 点云密度滑块：控制后端发布给网页端的最大点数
        if (this.cloudDensitySlider) {
            this.cloudDensitySlider.addEventListener('input', (e) => {
                const maxPoints = parseInt(e.target.value);

                this.cloudDensityValue.textContent = maxPoints;

                // 前端显示上限也同步调整
                if (this.pointCloudManager && this.pointCloudManager.setMaxPoints) {
                    this.pointCloudManager.setMaxPoints(maxPoints);
                }

                // 发布给后端 pointcloud_processor.py
                this.publishCloudDensity();

                this.log(`点云密度已设置为最大 ${maxPoints} 点`, 'info');
            });
        }

        // 累计帧数滑块：控制后端累计多少帧点云
        if (this.accumulationFramesSlider) {
            this.accumulationFramesSlider.addEventListener('input', (e) => {
                const frames = parseInt(e.target.value);

                this.accumulationFramesValue.textContent = frames;

                this.publishAccumulationFrames();

                this.log(`累计帧数已设置为 ${frames} 帧`, 'info');
            });
        }

        // 显示无人机复选框
        this.showDroneCheckbox.addEventListener('change', (e) => {
            if (this.droneMesh) {
                this.droneMesh.visible = e.target.checked;
            }
        });

        // ROI 控制按钮
        this.createRoiBtn.addEventListener('click', () => {
            this.createROI();
        });

        this.deleteRoiBtn.addEventListener('click', () => {
            this.deleteROI();
        });

        // 变换模式选择
        this.transformModeSelect.addEventListener('change', (e) => {
            if (this.roiSelector) {
                this.roiSelector.setTransformMode(e.target.value);
            }
        });

        // 裁剪控制按钮
        this.sendCropBtn.addEventListener('click', () => {
            this.sendCropBox();
        });

        this.resetCropBtn.addEventListener('click', () => {
            this.resetCrop();
        });

        // 曲面拟合按钮
        if (this.generateSurfaceBtn) {
            this.generateSurfaceBtn.addEventListener('click', () => {
                this.generateSurface();
            });
        }

        if (this.clearSurfaceBtn) {
            this.clearSurfaceBtn.addEventListener('click', () => {
                this.clearSurface();
            });
        }

        // 目标点设置按钮
        // UV 滑块事件监听
        this.startUSlider.addEventListener('input', (e) => {
            const value = parseFloat(e.target.value);
            this.startUValue.textContent = value.toFixed(2);
            this.publishStartUV();
        });

        this.startVSlider.addEventListener('input', (e) => {
            const value = parseFloat(e.target.value);
            this.startVValue.textContent = value.toFixed(2);
            this.publishStartUV();
        });

        this.goalUSlider.addEventListener('input', (e) => {
            const value = parseFloat(e.target.value);
            this.goalUValue.textContent = value.toFixed(2);
            this.publishGoalUV();
        });

        this.goalVSlider.addEventListener('input', (e) => {
            const value = parseFloat(e.target.value);
            this.goalVValue.textContent = value.toFixed(2);
            this.publishGoalUV();
        });

        // 执行规划按钮
        this.executePlanningBtn.addEventListener('click', () => {
            this.executePlanning();
        });

        // 重置按钮
        this.resetBtn.addEventListener('click', () => {
            this.resetAll();
        });

        // 鼠标点击事件（用于点选）
        this.renderer.domElement.addEventListener('click', (event) => {
            this.onCanvasClick(event);
        });

        // 键盘事件（ESC 取消点选）
        document.addEventListener('keydown', (event) => {
            if (event.key === 'Escape' && this.pickingMode) {
                this.exitPickingMode();
            }
        });
    }

    /**
     * 连接到 ROS
     */
    connect() {
        const url = this.rosUrlInput.value;
        this.log(`正在连接到 ${url}...`, 'info');

        this.ros = new ROSLIB.Ros({
            url: url
        });

        this.ros.on('connection', () => {
            this.isConnected = true;
            this.connectionStatus.className = 'status-dot connected';
            this.connectionText.textContent = '已连接';
            this.connectBtn.textContent = '断开连接';
            this.connectBtn.className = 'btn btn-danger';
            this.log('ROS 连接成功', 'success');

            // 初始化管理器和订阅
            this.initManagers();
            this.subscribeTopics();
        });

        this.ros.on('error', (error) => {
            this.log(`连接错误: ${error}`, 'error');
        });

        this.ros.on('close', () => {
            this.isConnected = false;
            this.connectionStatus.className = 'status-dot disconnected';
            this.connectionText.textContent = '未连接';
            this.connectBtn.textContent = '连接';
            this.connectBtn.className = 'btn btn-primary';
            this.log('ROS 连接已断开', 'warning');
        });
    }

    /**
     * 断开 ROS 连接
     */
    disconnect() {
        if (this.ros) {
            this.ros.close();
        }
    }

    /**
     * 初始化管理器
     */
    initManagers() {
        // 初始化点云管理器
        this.pointCloudManager = new PointCloudManager(this.ros, this.scene);

        // 初始化 ROI 选择器
        this.roiSelector = new ROISelector(
            this.scene,
            this.camera,
            this.renderer,
            this.transformControls
        );

        // 创建发布器
        this.roiPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planning/roi_box',
            messageType: 'geometry_msgs/PoseStamped'
        });

        this.goalPointsPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planning/goal_points',
            messageType: 'geometry_msgs/PoseArray'
        });

        // 裁剪框发布器 (发送给上位机进行点云裁剪)
        this.cropBoxPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planning/crop_box',
            messageType: 'geometry_msgs/PoseStamped'
        });

        // 裁剪框尺寸发布器
        this.cropBoxSizePublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planning/crop_box_size',
            messageType: 'geometry_msgs/Vector3'
        });

        // 裁剪重置发布器
        this.cropResetPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planning/crop_reset',
            messageType: 'std_msgs/Bool'
        });

        // 点云密度控制发布器
        this.cloudMaxPointsPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planning/cloud_max_points',
            messageType: 'std_msgs/Int32'
        });

        // 累计帧数控制发布器
        this.accumulationFramesPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planning/accumulation_frames',
            messageType: 'std_msgs/Int32'
        });

        // 曲面拟合触发发布器
        this.convertMeshTriggerPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/convert_mesh_trigger',
            messageType: 'std_msgs/Bool'
        });

        this.log('管理器初始化完成', 'info');
        this.publishCloudDensity();
        this.publishAccumulationFrames();
    }

    /**
     * 订阅 ROS Topics
     */
    subscribeTopics() {
        // 订阅点云
        const pointcloudTopic = this.pointcloudTopicInput.value;
        this.pointCloudManager.subscribe(pointcloudTopic);
        this.log(`已订阅点云: ${pointcloudTopic}`, 'info');

        // 订阅无人机位姿
        const poseTopic = this.poseTopicInput.value;
        this.poseSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: poseTopic,
            messageType: 'geometry_msgs/PoseStamped'
        });

        this.poseSubscriber.subscribe((message) => {
            this.updateDronePose(message);
        });

        this.log(`已订阅位姿: ${poseTopic}`, 'info');

        // 订阅起始点曲面点和法向量
        this.startSurfacePointSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/start_surface_point',
            messageType: 'geometry_msgs/PointStamped'
        });

        this.startSurfacePointSub.subscribe((message) => {
            this.updateStartSurfacePoint(message);
        });

        this.startSurfaceNormalSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/start_surface_normal',
            messageType: 'geometry_msgs/Vector3'
        });

        this.startSurfaceNormalSub.subscribe((message) => {
            this.updateStartSurfaceNormal(message);
        });

        // 订阅终点曲面点和法向量
        this.goalSurfacePointSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/goal_surface_point',
            messageType: 'geometry_msgs/PointStamped'
        });

        this.goalSurfacePointSub.subscribe((message) => {
            this.updateGoalSurfacePoint(message);
        });

        this.goalSurfaceNormalSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/goal_surface_normal',
            messageType: 'geometry_msgs/Vector3'
        });

        this.goalSurfaceNormalSub.subscribe((message) => {
            this.updateGoalSurfaceNormal(message);
        });

        // 订阅轨迹数据
        this.trajectorySub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/trajectory',
            messageType: 'visualization_msgs/MarkerArray'
        });

        this.trajectorySub.subscribe((message) => {
            this.updateTrajectory(message);
        });

        this.log('已订阅规划器 topics', 'info');

        // 订阅网格数据
        this.meshSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/mesh',
            messageType: 'visualization_msgs/MarkerArray'
        });

        this.meshSub.subscribe((message) => {
            this.updateMesh(message);
        });

        this.log('已订阅网格数据 topic', 'info');

        // 创建无人机模型
        this.createDroneModel();

        // 创建 UV 发布器
        this.setupUVPublishers();
    }

    /**
     * 创建无人机模型
     */
    createDroneModel() {
        if (this.droneMesh) {
            return;
        }

        // 创建简单的无人机模型（四个旋翼 + 机身）
        const droneGroup = new THREE.Group();

        // 机身
        const bodyGeometry = new THREE.BoxGeometry(0.3, 0.1, 0.3);
        const bodyMaterial = new THREE.MeshStandardMaterial({ color: 0xff6600 });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        droneGroup.add(body);

        // 四个旋翼臂
        const armGeometry = new THREE.CylinderGeometry(0.02, 0.02, 0.4);
        const armMaterial = new THREE.MeshStandardMaterial({ color: 0x333333 });

        const positions = [
            { x: 0.2, z: 0.2 },
            { x: -0.2, z: 0.2 },
            { x: 0.2, z: -0.2 },
            { x: -0.2, z: -0.2 }
        ];

        positions.forEach(pos => {
            const arm = new THREE.Mesh(armGeometry, armMaterial);
            arm.position.set(pos.x, 0, pos.z);
            arm.rotation.x = Math.PI / 2;
            droneGroup.add(arm);

            // 旋翼
            const propGeometry = new THREE.CylinderGeometry(0.15, 0.15, 0.02);
            const propMaterial = new THREE.MeshStandardMaterial({
                color: 0x666666,
                transparent: true,
                opacity: 0.5
            });
            const prop = new THREE.Mesh(propGeometry, propMaterial);
            prop.position.set(pos.x, 0, pos.z);
            droneGroup.add(prop);
        });

        // 方向指示器（前方箭头）
        const arrowGeometry = new THREE.ConeGeometry(0.05, 0.15, 8);
        const arrowMaterial = new THREE.MeshStandardMaterial({ color: 0xff0000 });
        const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
        arrow.position.set(0.2, 0, 0);
        arrow.rotation.z = -Math.PI / 2;
        droneGroup.add(arrow);

        this.droneMesh = droneGroup;
        this.droneMesh.name = 'drone';
        this.scene.add(this.droneMesh);
    }

    /**
     * 更新无人机位姿
     */
    updateDronePose(message) {
        const pose = message.pose;

        this.dronePosition = {
            x: pose.position.x,
            y: pose.position.y,
            z: pose.position.z
        };

        this.droneOrientation = {
            x: pose.orientation.x,
            y: pose.orientation.y,
            z: pose.orientation.z,
            w: pose.orientation.w
        };

        if (this.droneMesh) {
            this.droneMesh.position.set(
                this.dronePosition.x,
                this.dronePosition.y,
                this.dronePosition.z
            );

            this.droneMesh.quaternion.set(
                this.droneOrientation.x,
                this.droneOrientation.y,
                this.droneOrientation.z,
                this.droneOrientation.w
            );
        }
    }

    /**
     * 创建 ROI 裁剪框
     */
    createROI() {
        if (!this.isConnected) {
            this.log('请先连接到 ROS', 'warning');
            return;
        }

        console.log('创建 ROI 裁剪框...');
        console.log('transformControls:', this.transformControls);
        console.log('roiSelector:', this.roiSelector);

        this.roiSelector.create();

        // 确保 transformControls 附加到 roiBox 并设置模式
        if (this.transformControls && this.roiSelector.roiBox) {
            this.transformControls.attach(this.roiSelector.roiBox);
            const currentMode = this.transformModeSelect.value || 'translate';
            if (currentMode !== 'scale') {
                this.transformControls.setMode(currentMode);
            }
            console.log('TransformControls 已附加，模式:', currentMode);
        }

        this.createRoiBtn.disabled = true;
        this.deleteRoiBtn.disabled = false;
        this.sendCropBtn.disabled = false;
        this.roiInfo.style.display = 'block';
        this.updateROIInfo();
        this.updateExecuteButton();
        this.log('ROI 裁剪框已创建', 'success');
    }

    /**
     * 删除 ROI 裁剪框
     */
    deleteROI() {
        this.roiSelector.delete();
        this.createRoiBtn.disabled = false;
        this.deleteRoiBtn.disabled = true;
        this.sendCropBtn.disabled = true;
        this.roiInfo.style.display = 'none';
        this.updateExecuteButton();
        this.log('ROI 裁剪框已删除', 'info');
    }

    /**
     * 更新 ROI 信息显示
     */
    updateROIInfo() {
        // 安全检查
        if (!this.roiSelector || !this.roiSelector.isROIActive()) return;

        const params = this.roiSelector.getParameters();
        if (!params) return;

        const roiCenterEl = document.getElementById('roi-center');
        const roiSizeEl = document.getElementById('roi-size');

        if (roiCenterEl) {
            const centerText = '(' + params.center.x.toFixed(2) + ', ' +
                              params.center.y.toFixed(2) + ', ' +
                              params.center.z.toFixed(2) + ')';
            roiCenterEl.textContent = centerText;
        }

        if (roiSizeEl) {
            const sizeText = '(' + params.size.x.toFixed(2) + ', ' +
                            params.size.y.toFixed(2) + ', ' +
                            params.size.z.toFixed(2) + ')';
            roiSizeEl.textContent = sizeText;
        }

        const roiRotationEl = document.getElementById('roi-rotation');
        if (roiRotationEl) {
            const euler = new THREE.Euler().setFromQuaternion(
                new THREE.Quaternion(
                    params.orientation.x,
                    params.orientation.y,
                    params.orientation.z,
                    params.orientation.w
                )
            );

            const rotText = '(' + (euler.x * 180 / Math.PI).toFixed(1) + '°, ' +
                           (euler.y * 180 / Math.PI).toFixed(1) + '°, ' +
                           (euler.z * 180 / Math.PI).toFixed(1) + '°)';
            roiRotationEl.textContent = rotText;
        }
    }

    /**
     * 进入点选模式
     */
    enterPickingMode(mode) {
        if (!this.isConnected) {
            this.log('请先连接到 ROS', 'warning');
            return;
        }

        this.pickingMode = mode;

        // 显示提示并设置自动消失
        const overlay = this.pickingModeOverlay;
        overlay.style.display = 'block';
        overlay.style.opacity = '0.85';

        // 清除之前的定时器
        if (this.pickingOverlayTimer) {
            clearTimeout(this.pickingOverlayTimer);
        }

        // 3秒后淡出消失
        this.pickingOverlayTimer = setTimeout(() => {
            overlay.style.transition = 'opacity 0.5s ease';
            overlay.style.opacity = '0';
            setTimeout(() => {
                if (this.pickingMode) {
                    overlay.style.display = 'none';
                }
            }, 500);
        }, 3000);

        this.orbitControls.enabled = false;
        this.renderer.domElement.style.cursor = 'crosshair';

        const modeText = mode === 'start' ? '起始点' : '末端点';
        this.log(`进入${modeText}选择模式，点击点云表面选择目标点`, 'info');
    }

    /**
     * 退出点选模式
     */
    exitPickingMode() {
        this.pickingMode = null;

        // 清除定时器
        if (this.pickingOverlayTimer) {
            clearTimeout(this.pickingOverlayTimer);
            this.pickingOverlayTimer = null;
        }

        this.pickingModeOverlay.style.display = 'none';
        this.pickingModeOverlay.style.opacity = '0.85';
        this.pickingModeOverlay.style.transition = '';

        this.orbitControls.enabled = true;
        this.renderer.domElement.style.cursor = 'default';
        this.log('已退出点选模式', 'info');
    }

    /**
     * 画布点击事件处理
     */
    onCanvasClick(event) {
        if (!this.pickingMode) {
            return;
        }

        // 计算鼠标在标准化设备坐标中的位置
        const rect = this.renderer.domElement.getBoundingClientRect();
        this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

        // 更新射线
        this.raycaster.setFromCamera(this.mouse, this.camera);

        // 检测与点云的交互
        const point = this.pointCloudManager.getClosestPoint(this.raycaster);

        if (point) {
            if (this.pickingMode === 'start') {
                this.setStartPoint(point);
            } else if (this.pickingMode === 'end') {
                this.setEndPoint(point);
            }

            this.exitPickingMode();
        } else {
            this.log('未检测到点云，请点击点云表面', 'warning');
        }
    }

    /**
     * 设置起始点
     */
    setStartPoint(point) {
        this.startPoint = {
            x: point.x,
            y: point.y,
            z: point.z
        };

        // 移除旧标记
        if (this.startMarker) {
            this.scene.remove(this.startMarker);
        }

        // 创建新标记
        this.startMarker = this.createMarker(point, 0x00ff00);
        this.scene.add(this.startMarker);

        // 更新 UI
        const pointText = '(' + point.x.toFixed(2) + ', ' +
                         point.y.toFixed(2) + ', ' +
                         point.z.toFixed(2) + ')';
        this.startPointInfo.textContent = pointText;

        this.updateExecuteButton();
        this.log('起始点已设置', 'success');
    }

    /**
     * 设置末端点
     */
    setEndPoint(point) {
        this.endPoint = {
            x: point.x,
            y: point.y,
            z: point.z
        };

        // 移除旧标记
        if (this.endMarker) {
            this.scene.remove(this.endMarker);
        }

        // 创建新标记
        this.endMarker = this.createMarker(point, 0xff0000);
        this.scene.add(this.endMarker);

        // 更新 UI
        const pointText = '(' + point.x.toFixed(2) + ', ' +
                         point.y.toFixed(2) + ', ' +
                         point.z.toFixed(2) + ')';
        this.endPointInfo.textContent = pointText;

        this.updateExecuteButton();
        this.log('末端点已设置', 'success');
    }

    /**
     * 创建点标记
     */
    createMarker(point, color) {
        const markerGroup = new THREE.Group();

        // 球体标记
        const sphereGeometry = new THREE.SphereGeometry(0.05, 16, 16);
        const sphereMaterial = new THREE.MeshStandardMaterial({
            color: color,
            emissive: color,
            emissiveIntensity: 0.5
        });
        const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
        markerGroup.add(sphere);

        // 垂直线
        const lineGeometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(0, -point.z - 0.1, 0)
        ]);
        const lineMaterial = new THREE.LineBasicMaterial({
            color: color,
            linewidth: 2
        });
        const line = new THREE.Line(lineGeometry, lineMaterial);
        markerGroup.add(line);

        markerGroup.position.set(point.x, point.y, point.z);
        return markerGroup;
    }

    /**
     * 更新执行按钮状态
     */
    updateExecuteButton() {
        const hasROI = this.roiSelector && this.roiSelector.isROIActive();
        const hasStartPoint = this.startPoint !== null;
        const hasEndPoint = this.endPoint !== null;

        this.executeBtn.disabled = !(hasROI && hasStartPoint && hasEndPoint);
    }

    /**
     * 执行任务
     */
    executeTask() {
        if (!this.isConnected) {
            this.log('请先连接到 ROS', 'error');
            return;
        }

        // 获取 ROI 参数
        const roiParams = this.roiSelector.getParameters();
        if (!roiParams) {
            this.log('ROI 参数无效', 'error');
            return;
        }

        // 发布 ROI 盒子
        this.publishROI(roiParams);

        // 发布目标点
        this.publishGoalPoints();

        this.log('任务参数已发送到 ROS', 'success');
    }

    /**
     * 发布 ROI 到 ROS
     */
    publishROI(params) {
        const roiMessage = {
            header: {
                stamp: {
                    secs: Math.floor(Date.now() / 1000),
                    nsecs: (Date.now() % 1000) * 1000000
                },
                frame_id: 'web_frame'
            },
            pose: {
                position: {
                    x: params.center.x,
                    y: params.center.y,
                    z: params.center.z
                },
                orientation: {
                    x: params.orientation.x,
                    y: params.orientation.y,
                    z: params.orientation.z,
                    w: params.orientation.w
                }
            }
        };

        this.roiPublisher.publish(roiMessage);
        this.log('ROI 盒子已发布到 /planning/roi_box', 'info');

        // 同时发布尺寸信息（作为自定义消息或参数）
        // 注意：这里假设有一个额外的 topic 用于发布尺寸
        // 如果需要，可以创建一个自定义消息类型
        console.log('ROI 尺寸:', params.size);
    }

    /**
     * 发布目标点到 ROS
     */
    publishGoalPoints() {
        const goalMessage = {
            header: {
                stamp: {
                    secs: Math.floor(Date.now() / 1000),
                    nsecs: (Date.now() % 1000) * 1000000
                },
                frame_id: 'web_frame'
            },
            poses: [
                {
                    position: {
                        x: this.startPoint.x,
                        y: this.startPoint.y,
                        z: this.startPoint.z
                    },
                    orientation: {
                        x: 0,
                        y: 0,
                        z: 0,
                        w: 1
                    }
                },
                {
                    position: {
                        x: this.endPoint.x,
                        y: this.endPoint.y,
                        z: this.endPoint.z
                    },
                    orientation: {
                        x: 0,
                        y: 0,
                        z: 0,
                        w: 1
                    }
                }
            ]
        };

        this.goalPointsPublisher.publish(goalMessage);
        this.log('目标点已发布到 /planning/goal_points', 'info');
    }

    /**
     * 发送裁剪框到上位机
     */
    sendCropBox() {
        if (!this.isConnected) {
            this.log('请先连接到 ROS', 'error');
            return;
        }

        const roiParams = this.roiSelector.getParameters();
        if (!roiParams) {
            this.log('裁剪框参数无效', 'error');
            return;
        }

        // 发布裁剪框位姿
        const cropBoxMessage = {
            header: {
                stamp: {
                    secs: Math.floor(Date.now() / 1000),
                    nsecs: (Date.now() % 1000) * 1000000
                },
                frame_id: 'web_frame'  // 使用web_frame坐标系，与Three.js一致
            },
            pose: {
                position: {
                    x: roiParams.center.x,
                    y: roiParams.center.y,
                    z: roiParams.center.z
                },
                orientation: {
                    x: roiParams.orientation.x,
                    y: roiParams.orientation.y,
                    z: roiParams.orientation.z,
                    w: roiParams.orientation.w
                }
            }
        };

        this.cropBoxPublisher.publish(cropBoxMessage);

        // 发布裁剪框尺寸
        const sizeMessage = {
            x: roiParams.size.x,
            y: roiParams.size.y,
            z: roiParams.size.z
        };

        this.cropBoxSizePublisher.publish(sizeMessage);

        // 更新 UI 状态
        this.cropStatus.style.display = 'block';
        this.cropStatusText.textContent = '已发送裁剪框';
        this.cropStatusText.style.color = '#4CAF50';
        this.resetCropBtn.disabled = false;

        // 启用生产曲面按钮
        this.generateSurfaceBtn.disabled = false;

        // 强制刷新点云以获取裁剪后的数据
        if (this.pointCloudManager) {
            this.pointCloudManager.forceRefresh();
        }

        this.log(`裁剪框已发送: 中心(${roiParams.center.x.toFixed(2)}, ${roiParams.center.y.toFixed(2)}, ${roiParams.center.z.toFixed(2)}), 尺寸(${roiParams.size.x.toFixed(2)}, ${roiParams.size.y.toFixed(2)}, ${roiParams.size.z.toFixed(2)})`, 'success');
    }

    /**
     * 重置裁剪
     */
    resetCrop() {
        if (!this.isConnected) {
            this.log('请先连接到 ROS', 'error');
            return;
        }

        // 发布重置命令
        const resetMessage = {
            data: true
        };

        this.cropResetPublisher.publish(resetMessage);

        // 更新 UI 状态
        this.cropStatusText.textContent = '已重置裁剪';
        this.cropStatusText.style.color = '#FFB74D';
        this.resetCropBtn.disabled = true;

        // 强制刷新点云以获取重置后的数据
        if (this.pointCloudManager) {
            this.pointCloudManager.forceRefresh();
        }

        this.log('已发送裁剪重置命令', 'info');
    }

    /**
     * 重置所有
     */
    resetAll() {
        // 删除 ROI
        if (this.roiSelector && this.roiSelector.isROIActive()) {
            this.deleteROI();
        }

        // 清除起始点和终点的可视化
        if (this.startMarker) {
            this.scene.remove(this.startMarker);
            this.startMarker = null;
        }
        if (this.startNormalArrow) {
            this.scene.remove(this.startNormalArrow);
            this.startNormalArrow = null;
        }
        if (this.goalMarker) {
            this.scene.remove(this.goalMarker);
            this.goalMarker = null;
        }
        if (this.goalNormalArrow) {
            this.scene.remove(this.goalNormalArrow);
            this.goalNormalArrow = null;
        }

        // 清除轨迹
        if (this.trajectoryGroup) {
            this.scene.remove(this.trajectoryGroup);
            this.trajectoryGroup = null;
        }

        this.startPointInfo.textContent = '未设置';
        this.goalPointInfo.textContent = '未设置';

        this.log('已重置所有设置', 'info');
    }

    /**
     * 设置 UV 发布器
     */
    setupUVPublishers() {
        // 创建起始点 UV 发布器
        this.startUVPub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/start_uv',
            messageType: 'geometry_msgs/Vector3'
        });

        // 创建终点 UV 发布器
        this.goalUVPub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/goal_uv',
            messageType: 'geometry_msgs/Vector3'
        });

        // 创建规划触发发布器
        this.planTriggerPub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/planner/plan_trigger',
            messageType: 'std_msgs/Bool'
        });

        // 初始化时发布一次 UV 参数
        this.publishStartUV();
        this.publishGoalUV();
    }

    /**
     * 发布点云密度设置到后端 pointcloud_processor.py
     */
    publishCloudDensity() {
        if (!this.cloudDensitySlider) {
            return;
        }

        const maxPoints = parseInt(this.cloudDensitySlider.value);

        if (this.cloudDensityValue) {
            this.cloudDensityValue.textContent = maxPoints;
        }

        // 同步前端 pointcloud.js 的显示上限
        if (this.pointCloudManager && this.pointCloudManager.setMaxPoints) {
            this.pointCloudManager.setMaxPoints(maxPoints);
        }

        if (!this.isConnected || !this.cloudMaxPointsPublisher) {
            return;
        }

        const message = new ROSLIB.Message({
            data: maxPoints
        });

        this.cloudMaxPointsPublisher.publish(message);
    }

    /**
     * 发布累计帧数设置到后端 pointcloud_processor.py
     */
    publishAccumulationFrames() {
        if (!this.accumulationFramesSlider) {
            return;
        }

        const frames = parseInt(this.accumulationFramesSlider.value);

        if (this.accumulationFramesValue) {
            this.accumulationFramesValue.textContent = frames;
        }

        if (!this.isConnected || !this.accumulationFramesPublisher) {
            return;
        }

        const message = new ROSLIB.Message({
            data: frames
        });

        this.accumulationFramesPublisher.publish(message);
    }

    /**
     * 发布起始点 UV 参数
     */
    publishStartUV() {
        if (!this.startUVPub) return;

        const u = parseFloat(this.startUSlider.value);
        const v = parseFloat(this.startVSlider.value);

        const message = new ROSLIB.Message({
            x: u,
            y: v,
            z: 0.0
        });

        this.startUVPub.publish(message);
    }

    /**
     * 发布终点 UV 参数
     */
    publishGoalUV() {
        if (!this.goalUVPub) return;

        const u = parseFloat(this.goalUSlider.value);
        const v = parseFloat(this.goalVSlider.value);

        const message = new ROSLIB.Message({
            x: u,
            y: v,
            z: 0.0
        });

        this.goalUVPub.publish(message);
    }

    /**
     * 更新起始点曲面点
     */
    updateStartSurfacePoint(message) {
        const point = message.point;

        // 移除旧的标记
        if (this.startMarker) {
            this.scene.remove(this.startMarker);
        }

        // 创建新的球体标记
        const geometry = new THREE.SphereGeometry(0.05, 16, 16);
        const material = new THREE.MeshStandardMaterial({ color: 0x00ff00 });
        this.startMarker = new THREE.Mesh(geometry, material);
        this.startMarker.position.set(point.x, point.y, point.z);
        this.scene.add(this.startMarker);

        // 更新信息显示
        this.startPointInfo.textContent = `(${point.x.toFixed(2)}, ${point.y.toFixed(2)}, ${point.z.toFixed(2)})`;

        // 启用执行按钮
        this.updatePlanningButton();
    }

    /**
     * 更新起始点法向量
     */
    updateStartSurfaceNormal(message) {
        if (!this.startMarker) return;

        // 移除旧的箭头
        if (this.startNormalArrow) {
            this.scene.remove(this.startNormalArrow);
        }

        // 创建法向量箭头
        const origin = this.startMarker.position;
        const direction = new THREE.Vector3(message.x, message.y, message.z).normalize();
        const length = 0.2;
        const color = 0x0000ff;

        this.startNormalArrow = new THREE.ArrowHelper(direction, origin, length, color, 0.05, 0.03);
        this.scene.add(this.startNormalArrow);
    }

    /**
     * 更新终点曲面点
     */
    updateGoalSurfacePoint(message) {
        const point = message.point;

        // 移除旧的标记
        if (this.goalMarker) {
            this.scene.remove(this.goalMarker);
        }

        // 创建新的球体标记
        const geometry = new THREE.SphereGeometry(0.05, 16, 16);
        const material = new THREE.MeshStandardMaterial({ color: 0xff0000 });
        this.goalMarker = new THREE.Mesh(geometry, material);
        this.goalMarker.position.set(point.x, point.y, point.z);
        this.scene.add(this.goalMarker);

        // 更新信息显示
        this.goalPointInfo.textContent = `(${point.x.toFixed(2)}, ${point.y.toFixed(2)}, ${point.z.toFixed(2)})`;

        // 启用执行按钮
        this.updatePlanningButton();
    }

    /**
     * 更新终点法向量
     */
    updateGoalSurfaceNormal(message) {
        if (!this.goalMarker) return;

        // 移除旧的箭头
        if (this.goalNormalArrow) {
            this.scene.remove(this.goalNormalArrow);
        }

        // 创建法向量箭头
        const origin = this.goalMarker.position;
        const direction = new THREE.Vector3(message.x, message.y, message.z).normalize();
        const length = 0.2;
        const color = 0x0000ff;

        this.goalNormalArrow = new THREE.ArrowHelper(direction, origin, length, color, 0.05, 0.03);
        this.scene.add(this.goalNormalArrow);
    }

    /**
     * 更新规划按钮状态
     */
    updatePlanningButton() {
        const hasStartPoint = this.startMarker !== null && this.startMarker !== undefined;
        const hasGoalPoint = this.goalMarker !== null && this.goalMarker !== undefined;
        this.executePlanningBtn.disabled = !(hasStartPoint && hasGoalPoint);
    }

    /**
     * 执行轨迹规划
     */
    executePlanning() {
        if (!this.planTriggerPub) {
            this.log('规划触发器未初始化', 'error');
            return;
        }

        this.log('开始执行轨迹规划...', 'info');

        const message = new ROSLIB.Message({
            data: true
        });

        this.planTriggerPub.publish(message);
    }

    /**
     * 更新轨迹显示
     */
    updateTrajectory(message) {
        // 移除旧的轨迹
        if (this.trajectoryGroup) {
            this.scene.remove(this.trajectoryGroup);
        }

        this.trajectoryGroup = new THREE.Group();

        // 解析 MarkerArray
        for (const marker of message.markers) {
            if (marker.ns === 'surface_trajectory') {
                // 曲面轨迹线
                const points = [];
                for (const point of marker.points) {
                    points.push(new THREE.Vector3(point.x, point.y, point.z));
                }

                const geometry = new THREE.BufferGeometry().setFromPoints(points);
                const material = new THREE.LineBasicMaterial({
                    color: new THREE.Color(marker.color.r, marker.color.g, marker.color.b),
                    linewidth: 2
                });
                const line = new THREE.Line(geometry, material);
                this.trajectoryGroup.add(line);

            } else if (marker.ns === 'surface_normals') {
                // 法向量箭头
                if (marker.points.length >= 2) {
                    const start = marker.points[0];
                    const end = marker.points[1];
                    const origin = new THREE.Vector3(start.x, start.y, start.z);
                    const direction = new THREE.Vector3(
                        end.x - start.x,
                        end.y - start.y,
                        end.z - start.z
                    ).normalize();
                    const length = origin.distanceTo(new THREE.Vector3(end.x, end.y, end.z));
                    const color = new THREE.Color(marker.color.r, marker.color.g, marker.color.b);

                    const arrow = new THREE.ArrowHelper(direction, origin, length, color.getHex(), 0.02, 0.015);
                    this.trajectoryGroup.add(arrow);
                }
            }
        }

        this.scene.add(this.trajectoryGroup);
        this.log(`轨迹已更新: ${message.markers.length} 个标记`, 'success');
    }

    /**
     * 动画循环
     */
    animate() {
        requestAnimationFrame(() => this.animate());

        // 更新控制器
        this.orbitControls.update();

        // 更新缩放控制点位置（如果在缩放模式下）
        if (this.roiSelector && this.roiSelector.isScaleMode) {
            this.roiSelector.updateScaleHandlePositions();
        }

        // 渲染场景
        this.renderer.render(this.scene, this.camera);
    }

    /**
     * 日志记录
     */
    log(message, type = 'info') {
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry ${type}`;
        logEntry.textContent = `[${timestamp}] ${message}`;

        this.logContainer.appendChild(logEntry);
        this.logContainer.scrollTop = this.logContainer.scrollHeight;

        // 限制日志条目数量
        while (this.logContainer.children.length > 100) {
            this.logContainer.removeChild(this.logContainer.firstChild);
        }
    }

    /**
     * 切换全屏模式
     */
    toggleFullscreen() {
        if (!document.fullscreenElement && !document.webkitFullscreenElement) {
            // 进入全屏
            const elem = document.documentElement;
            if (elem.requestFullscreen) {
                elem.requestFullscreen();
            } else if (elem.webkitRequestFullscreen) {
                elem.webkitRequestFullscreen();
            }
            this.log('进入全屏模式', 'info');
        } else {
            // 退出全屏
            if (document.exitFullscreen) {
                document.exitFullscreen();
            } else if (document.webkitExitFullscreen) {
                document.webkitExitFullscreen();
            }
            this.log('退出全屏模式', 'info');
        }
    }

    /**
     * 更新全屏按钮图标
     */
    updateFullscreenButton() {
        if (this.fullscreenBtn) {
            if (document.fullscreenElement || document.webkitFullscreenElement) {
                this.fullscreenBtn.textContent = '⛶';
                this.fullscreenBtn.title = '退出全屏';
            } else {
                this.fullscreenBtn.textContent = '⛶';
                this.fullscreenBtn.title = '全屏显示';
            }

            // 触发窗口大小调整以更新渲染器
            setTimeout(() => {
                const viewport = document.getElementById('viewport');
                if (this.camera && this.renderer && viewport) {
                    this.camera.aspect = viewport.clientWidth / viewport.clientHeight;
                    this.camera.updateProjectionMatrix();
                    this.renderer.setSize(viewport.clientWidth, viewport.clientHeight);
                }
            }, 100);
        }
    }

    /**
     * 生产曲面
     */
    generateSurface() {
        console.log('generateSurface 被调用');
        console.log('isConnected:', this.isConnected);
        console.log('convertMeshTriggerPublisher:', this.convertMeshTriggerPublisher);

        if (!this.isConnected) {
            this.log('请先连接到 ROS', 'error');
            return;
        }

        if (!this.convertMeshTriggerPublisher) {
            this.log('曲面拟合发布器未初始化', 'error');
            return;
        }

        this.log('正在生产曲面...', 'info');

        // 发送触发信号到 gRPC bridge
        const message = new ROSLIB.Message({
            data: true
        });

        this.convertMeshTriggerPublisher.publish(message);

        // 更新 UI 状态
        this.surfaceStatusText.textContent = '正在拟合...';
        this.surfaceStatusText.style.color = '#FFB74D';
        this.generateSurfaceBtn.disabled = true;

        this.log('已发送曲面拟合请求', 'success');
    }

    /**
     * 取消曲面
     */
    clearSurface() {
        // 移除网格可视化
        if (this.meshGroup) {
            this.scene.remove(this.meshGroup);
            this.meshGroup = null;
        }

        // 更新状态
        this.surfaceFitted = false;
        this.surfaceStatusText.textContent = '未拟合';
        this.surfaceStatusText.style.color = '#666';
        this.clearSurfaceBtn.disabled = true;
        this.generateSurfaceBtn.disabled = false;

        this.log('已清除曲面', 'info');
    }

    /**
     * 更新网格显示
     */
    updateMesh(message) {
        try {
            // 移除旧的网格
            if (this.meshGroup) {
                this.scene.remove(this.meshGroup);
            }

            this.meshGroup = new THREE.Group();

            // 解析 MarkerArray
            for (const marker of message.markers) {
                if (marker.ns === 'surface_mesh' && marker.type === 11) { // TRIANGLE_LIST = 11
                    // 创建网格几何体
                    const geometry = new THREE.BufferGeometry();
                    const vertices = [];

                    // 提取顶点
                    for (const point of marker.points) {
                        vertices.push(point.x, point.y, point.z);
                    }

                    geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
                    geometry.computeVertexNormals();

                    // 创建材质
                    const material = new THREE.MeshStandardMaterial({
                        color: new THREE.Color(marker.color.r, marker.color.g, marker.color.b),
                        transparent: true,
                        opacity: marker.color.a,
                        side: THREE.DoubleSide,
                        flatShading: false
                    });

                    // 创建网格
                    const mesh = new THREE.Mesh(geometry, material);
                    this.meshGroup.add(mesh);
                }
            }

            this.scene.add(this.meshGroup);

            // 更新状态
            this.surfaceFitted = true;
            this.surfaceStatusText.textContent = '已拟合';
            this.surfaceStatusText.style.color = '#4CAF50';
            this.clearSurfaceBtn.disabled = false;
            this.generateSurfaceBtn.disabled = false;

            this.log(`曲面网格已更新: ${message.markers.length} 个标记`, 'success');

        } catch (error) {
            this.log(`更新网格失败: ${error}`, 'error');
        }
    }
}

// 启动应用
document.addEventListener('DOMContentLoaded', () => {
    const app = new ROSPointCloudApp();
    window.app = app; // 用于调试
});
