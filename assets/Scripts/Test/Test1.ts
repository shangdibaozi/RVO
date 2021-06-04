
import { _decorator, Component, Node, v3, Vec3, instantiate, systemEvent, SystemEventType, EventMouse, PhysicsSystem, PhysicsRayResult, Camera, geometry, log, CCInteger } from 'cc';
const { Ray } = geometry;
import { RVOMath, Vector2 } from '../RVO/Common';
import { Simulator } from '../RVO/Simulator';
const { ccclass, property } = _decorator;

@ccclass('Test1')
export class Test1 extends Component {
    @property(Camera)
    camera: Camera;

    @property(Node)
    avatarLayer: Node;

    @property(Node)
    sphereBlue: Node;

    @property(Node)
    sphereRed: Node;

    @property(CCInteger)
    agentCnt = 10;

    space = 3;
    speed = 6;
    goals: Vector2[] = [];
    mSpheres: Node[] = [];
    
    private _ray = new Ray();

    onEnable() {
        systemEvent.on(SystemEventType.MOUSE_DOWN, this.onMouseDown, this);
    }

    onDisable() {
        systemEvent.off(SystemEventType.MOUSE_DOWN, this.onMouseDown, this);
    }

    start () {
        let simulator = Simulator.instance;

        simulator.setAgentDefaults(6, 4, 1, 0.1, 0.5, this.speed, new Vector2(0, 0));

        this.createAgent(v3(-20, 0, 0), this.sphereBlue, 1);
        this.createAgent(v3(20, 0, 0), this.sphereRed, 1);

        this.createGameObject(v3(0, 0, -10), this.sphereRed, 6, 100000);
    }

    onMouseDown(event: EventMouse) {
        this.camera.screenPointToRay(event.getLocationX(), event.getLocationY(), this._ray);
        let worldPosition: Vec3 = null; 
        
        if(PhysicsSystem.instance.raycast(this._ray)) {
            const r = PhysicsSystem.instance.raycastResults;
            let minDistance = Number.MAX_VALUE;
            let item: PhysicsRayResult = null;
            for(let i = 0; i < r.length; i++) {
                if(r[i].distance < minDistance) {
                    minDistance = r[i].distance;
                    item = r[i];
                }
            }
            worldPosition = item.hitPoint;
        }

        let code = event.getButton();

        if(code === EventMouse.BUTTON_LEFT) {
            let index = 0;
            for (let i = 0; i < this.agentCnt; i++) {
                for (let j = 0; j < this.agentCnt; j++) {
                    let p = this.goals[index++];
                    p.x = i * this.space + worldPosition.x;
                    p.y = j * this.space + worldPosition.z;
                }
            }
        }
        else if(code === EventMouse.BUTTON_RIGHT) {
            let index = this.agentCnt * this.agentCnt;
            for (let i = 0; i < this.agentCnt; i++) {
                for (let j = 0; j < this.agentCnt; j++) {
                    let p = this.goals[index++];
                    p.x = i * this.space + worldPosition.x;
                    p.y = j * this.space + worldPosition.z;
                }
            }
        }
    }

    createAgent(pos: Vec3, prefab: Node, mass: number) {
        let cnt = this.agentCnt;
        let simulator = Simulator.instance;
        for(let i = 0; i < cnt; i++) {
            for(let j = 0; j < cnt; j++) {
                let p = new Vector2(i * this.space + pos.x, j * this.space + pos.z);
                let idx = simulator.addAgent(p);
                simulator.setAgentMass(idx, mass);
                this.goals.push(p);
                let node = instantiate(prefab);
                this.mSpheres.push(node);
                node.setWorldPosition(p.x, 0, p.y);
                node.parent = this.avatarLayer;
            }
        }
    }

    createGameObject(position: Vec3, spherePrefab: Node, radius: number, mass: number) {
        // Simulator.instance.setAgentDefaults(radius * 10, 4, 0.5, 1, radius * 10, this.speed, new Vector2(0, 0));
        Simulator.instance.setAgentDefaults(10, 10, 1, 1, radius, this.speed, new Vector2(0, 0));

        let p = new Vector2(position.x, position.z);
        let idx = Simulator.instance.addAgent(p);
        Simulator.instance.setAgentMass(idx, mass);

        this.goals.push(p);

        let g = instantiate(spherePrefab);
        g.setWorldScale(new Vec3(radius, radius, radius));
        this.mSpheres.push(g);
        g.parent = this.avatarLayer;
    }

    setPreferredVelocities() {
        let agentCnt = Simulator.instance.getNumAgents();
        for(let i = 0; i < agentCnt; i++) {
            let goalVector = this.goals[i].minus(Simulator.instance.getAgentPosition(i));
            if(RVOMath.absSq(goalVector) > 1.0) {
                goalVector = RVOMath.normalize(goalVector).scale(this.speed);
            }
            if (RVOMath.absSq(goalVector) < RVOMath.RVO_EPSILON) {
                // Agent is within one radius of its goal, set preferred velocity to zero
                Simulator.instance.setAgentPrefVelocity (i, new Vector2 (0.0, 0.0));
            }
            else {
                Simulator.instance.setAgentPrefVelocity(i, goalVector);
    
                let angle = Math.random() * 2.0 * Math.PI;
                let dist = Math.random() * 0.0001;
                Simulator.instance.setAgentPrefVelocity(i,
                    Simulator.instance.getAgentPrefVelocity(i).plus(new Vector2(Math.cos(angle), Math.sin(angle)).scale(dist)));
            }
        }
    }

    update(dt: number) {
        // 更新逻辑坐标
        this.setPreferredVelocities();
        Simulator.instance.run(dt);

        // 更新渲染坐标
        for(let i = 0; i < Simulator.instance.getNumAgents(); i++) {
            let p = Simulator.instance.getAgentPosition(i);
            this.mSpheres[i].setWorldPosition(p.x, 0, p.y);
        }
    }
}