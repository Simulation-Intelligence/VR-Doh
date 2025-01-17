import * as THREE from 'three';
import { OrbitControls } from 'OrbitControls';
import { OBJLoader } from 'OBJLoader';
import { MTLLoader } from 'MTLLoader';


export function initModelViewer(containerId, objPath, mtlPath) {
    const container = document.getElementById(containerId);
    if (!container) {
        console.error(`Container with id "${containerId}" not found.`);
        return;
    }

    // 创建场景
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xeeeeee);

    // 创建相机
    const camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
    camera.position.z = 0.7;

    // 渲染器
    const renderer = new THREE.WebGLRenderer();
    renderer.gammaOutput = true;  // 启用伽马输出
    renderer.gammaFactor = 2.2;   // 设置伽马值为2.2
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    // 添加光源
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 7.5);
    scene.add(directionalLight);

    // 控制器
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;

    // 加载材质文件 (.mtl)
    const mtlLoader = new MTLLoader();
    mtlLoader.load(
        mtlPath,
        (materials) => {
            materials.preload();

            // 加载模型文件 (.obj)
            const objLoader = new OBJLoader();
            objLoader.setMaterials(materials);
            objLoader.load(
                objPath,
                (object) => {
                    scene.add(object);
                    console.log('Model with materials loaded successfully');
                },
                (xhr) => {
                    console.log(`Loading: ${(xhr.loaded / xhr.total * 100).toFixed(2)}%`);
                },
                (error) => {
                    console.error('An error occurred while loading the model:', error);
                }
            );
        },
        (xhr) => {
            console.log(`Loading materials: ${(xhr.loaded / xhr.total * 100).toFixed(2)}%`);
        },
        (error) => {
            console.error('An error occurred while loading the materials:', error);
        }
    );

    // 自适应窗口大小
    window.addEventListener('resize', () => {
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
    });

    // 动画循环
    function animate() {
        requestAnimationFrame(animate);
        controls.update();
        renderer.render(scene, camera);
    }
    animate();
}
