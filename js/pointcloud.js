/**
 * 点云处理模块
 * 负责订阅和渲染 ROS PointCloud2 消息
 */

class PointCloudManager {
    constructor(ros, scene) {
        this.ros = ros;
        this.scene = scene;
        this.pointCloudTopic = null;
        this.pointCloudSubscriber = null;
        this.pointsMesh = null;
        this.pointsGeometry = null;
        this.pointsMaterial = null;

        // 移动设备检测
        this.isMobile = this.detectMobile();
        this.isAndroid = /Android/i.test(navigator.userAgent);
        this.androidVersion = this.getAndroidVersion();

        // 根据设备类型调整点云限制
        // 安卓9及以下设备使用更低的限制
        if (this.isAndroid && this.androidVersion <= 9) {
            this.maxPoints = 50000; // 安卓9设备限制为5万点
            this.updateInterval = 200; // 更长的更新间隔
            console.log('检测到安卓9设备，使用低性能模式');
        } else if (this.isMobile) {
            this.maxPoints = 100000; // 其他移动设备限制为10万点
            this.updateInterval = 150;
            console.log('检测到移动设备，使用移动端优化模式');
        } else {
            this.maxPoints = 500000; // 桌面设备50万点
            this.updateInterval = 100;
        }

        this.pointSize = 0.05; // 默认点大小
        this.lastUpdateTime = 0;

        // 点云数据缓存
        this.pointsData = {
            positions: [],
            colors: []
        };
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
     * 订阅点云 Topic
     */
    subscribe(topicName) {
        // 取消之前的订阅
        if (this.pointCloudSubscriber) {
            this.pointCloudSubscriber.unsubscribe();
        }

        this.pointCloudTopic = topicName;

        // 创建新的订阅
        this.pointCloudSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: topicName,
            messageType: 'sensor_msgs/PointCloud2',
            throttle_rate: 200 // 限制更新频率
        });

        this.pointCloudSubscriber.subscribe((message) => {
            this.handlePointCloudMessage(message);
        });

        console.log(`已订阅点云 Topic: ${topicName}`);
    }

    /**
     * 处理点云消息
     */
    handlePointCloudMessage(message) {
        const now = Date.now();
        if (now - this.lastUpdateTime < this.updateInterval) {
            return; // 跳过过于频繁的更新
        }
        this.lastUpdateTime = now;

        try {
            // 解析 PointCloud2 消息
            const points = this.parsePointCloud2(message);

            if (points.positions.length > 0) {
                this.updatePointCloud(points);
            }
        } catch (error) {
            console.error('解析点云数据失败:', error);
        }
    }

    /**
     * Base64 解码函数
     */
    base64ToUint8Array(base64) {
        const binaryString = atob(base64);
        const len = binaryString.length;
        const bytes = new Uint8Array(len);
        for (let i = 0; i < len; i++) {
            bytes[i] = binaryString.charCodeAt(i);
        }
        return bytes;
    }

    /**
     * 解析 PointCloud2 消息格式
     */
    parsePointCloud2(message) {
        const positions = [];
        const colors = [];

        // 获取字段信息
        const fields = message.fields;
        const pointStep = message.point_step;

        // rosbridge 将二进制数据编码为 base64 字符串
        let data;
        if (typeof message.data === 'string') {
            // Base64 编码的字符串，需要解码
            data = this.base64ToUint8Array(message.data);
        } else if (message.data instanceof Array) {
            // 数组格式
            data = new Uint8Array(message.data);
        } else {
            // 已经是 Uint8Array
            data = message.data;
        }

        console.log(`点云数据: ${data.length} 字节, point_step: ${pointStep}, 预计点数: ${Math.floor(data.length / pointStep)}`);

        // 查找 x, y, z, rgb 字段的偏移量
        let xOffset = -1, yOffset = -1, zOffset = -1, rgbOffset = -1;
        let xDatatype = 0, yDatatype = 0, zDatatype = 0;

        for (let field of fields) {
            if (field.name === 'x') {
                xOffset = field.offset;
                xDatatype = field.datatype;
            } else if (field.name === 'y') {
                yOffset = field.offset;
                yDatatype = field.datatype;
            } else if (field.name === 'z') {
                zOffset = field.offset;
                zDatatype = field.datatype;
            } else if (field.name === 'rgb' || field.name === 'rgba') {
                rgbOffset = field.offset;
            }
        }

        if (xOffset === -1 || yOffset === -1 || zOffset === -1) {
            console.error('点云数据缺少 x, y, z 字段');
            return { positions, colors };
        }

        // 计算点数
        const numPoints = Math.min(
            Math.floor(data.length / pointStep),
            this.maxPoints
        );

        // 采样间隔（如果点太多，进行下采样）
        const step = Math.max(1, Math.floor(numPoints / this.maxPoints));

        // 解析每个点
        for (let i = 0; i < numPoints; i += step) {
            const offset = i * pointStep;

            // 读取 x, y, z 坐标（假设是 float32）
            // ROS 坐标系: X-前, Y-左, Z-上
            // Three.js 坐标系: X-右, Y-上, Z-前
            // 转换: Three.x = ROS.x, Three.y = ROS.z, Three.z = -ROS.y
            const rosX = this.readFloat32(data, offset + xOffset);
            const rosY = this.readFloat32(data, offset + yOffset);
            const rosZ = this.readFloat32(data, offset + zOffset);

            const x = rosX;
            const y = rosZ;  // ROS Z -> Three.js Y (上)
            const z = -rosY; // ROS Y -> Three.js -Z

            // 检查是否为有效值
            if (isNaN(x) || isNaN(y) || isNaN(z) ||
                !isFinite(x) || !isFinite(y) || !isFinite(z)) {
                continue;
            }

            positions.push(x, y, z);

            // 读取颜色（如果有）
            if (rgbOffset !== -1) {
                const rgb = this.readUint32(data, offset + rgbOffset);
                const r = ((rgb >> 16) & 0xFF) / 255.0;
                const g = ((rgb >> 8) & 0xFF) / 255.0;
                const b = (rgb & 0xFF) / 255.0;
                colors.push(r, g, b);
            } else {
                // 默认颜色（根据高度着色）
                const heightColor = (z + 2) / 4; // 假设高度范围 -2 到 2
                colors.push(heightColor, 1 - heightColor, 0.5);
            }
        }

        return { positions, colors };
    }

    /**
     * 从字节数组读取 float32
     */
    readFloat32(data, offset) {
        const view = new DataView(data.buffer, data.byteOffset + offset, 4);
        return view.getFloat32(0, true); // little-endian
    }

    /**
     * 从字节数组读取 uint32
     */
    readUint32(data, offset) {
        const view = new DataView(data.buffer, data.byteOffset + offset, 4);
        return view.getUint32(0, true); // little-endian
    }

    /**
     * 更新点云渲染
     */
    updatePointCloud(points) {
        // 如果还没有创建点云对象，先创建
        if (!this.pointsMesh) {
            this.createPointCloudMesh();
        }

        // 更新几何体
        this.pointsGeometry.setAttribute(
            'position',
            new THREE.Float32BufferAttribute(points.positions, 3)
        );

        this.pointsGeometry.setAttribute(
            'color',
            new THREE.Float32BufferAttribute(points.colors, 3)
        );

        this.pointsGeometry.computeBoundingSphere();
        this.pointsGeometry.attributes.position.needsUpdate = true;
        this.pointsGeometry.attributes.color.needsUpdate = true;

        // 缓存数据用于点选
        this.pointsData = points;
    }

    /**
     * 创建点云网格对象
     */
    createPointCloudMesh() {
        this.pointsGeometry = new THREE.BufferGeometry();

        this.pointsMaterial = new THREE.PointsMaterial({
            size: this.pointSize,
            vertexColors: true,
            sizeAttenuation: true
        });

        this.pointsMesh = new THREE.Points(this.pointsGeometry, this.pointsMaterial);
        this.pointsMesh.name = 'pointcloud';
        this.scene.add(this.pointsMesh);
    }

    /**
     * 设置点的大小
     */
    setPointSize(size) {
        this.pointSize = size;
        if (this.pointsMaterial) {
            this.pointsMaterial.size = size;
        }
    }

    /**
     * 获取最近的点（用于点选）
     */
    getClosestPoint(raycaster) {
        if (!this.pointsMesh) {
            return null;
        }

        const intersects = raycaster.intersectObject(this.pointsMesh);

        if (intersects.length > 0) {
            return intersects[0].point;
        }

        return null;
    }

    /**
     * 清理资源
     */
    dispose() {
        if (this.pointCloudSubscriber) {
            this.pointCloudSubscriber.unsubscribe();
        }

        if (this.pointsMesh) {
            this.scene.remove(this.pointsMesh);
            this.pointsGeometry.dispose();
            this.pointsMaterial.dispose();
        }
    }
}
