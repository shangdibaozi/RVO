import { RVOMath, Obstacle, Vector2 } from "./Common";
import { Agent } from "./Agent";
import { KdTree } from "./kdtree";


export class Simulator {
    private agentId: number = 0;
    private agentIdLst: number[] = [];
    aid2agent: {[key: string]: Agent} = Object.create(null);
    
	obstacles: Obstacle[] = [];
	kdTree: KdTree = new KdTree();
	
	
    defaultAgent: Agent; // Agent
    time: number = 0.0;

    private static _inst: Simulator;
    static get instance(): Simulator {
        if(!Simulator._inst) {
            Simulator._inst = new Simulator(); 
        }
        return Simulator._inst;
    }

    getAgent(idx: number) {
        return this.aid2agent[this.agentIdLst[idx]];
    }

    getAgentByAid(aid: number) {
        return this.aid2agent[aid];
    }

    getGlobalTime() {
    	return this.time;
    };
    
    getNumAgents() {
    	return this.agentIdLst.length;
    };


    setAgentPrefVelocity(aid: number, velocity: Vector2) {
        this.aid2agent[aid].prefVelocity_.copy(velocity);
    }
    
    getAgentPosition(aid: number) {
        return this.aid2agent[aid].position_;
    }
    
    getAgentPrefVelocity(aid: number) {
        return this.aid2agent[aid].prefVelocity_;
    }
    
    getAgentVelocity(aid: number) {
        return this.aid2agent[aid].velocity_;
    }
    
    getAgentRadius(aid: number) {
        return this.aid2agent[aid].radius_;
    }
    
    getAgentOrcaLines(aid: number) {
        return this.aid2agent[aid].orcaLines_;
    }

    addAgent(position: Vector2) {
        if (!this.defaultAgent) {
            throw new Error("no default agent");
        }

        let agent = new Agent();
        
        agent.position_.copy(position);
        agent.maxNeighbors_ = this.defaultAgent.maxNeighbors_;
        agent.maxSpeed_ = this.defaultAgent.maxSpeed_;
        agent.neighborDist = this.defaultAgent.neighborDist;
        agent.radius_ = this.defaultAgent.radius_;
        agent.timeHorizon = this.defaultAgent.timeHorizon;
        agent.timeHorizonObst = this.defaultAgent.timeHorizonObst;
        agent.velocity_.copy(this.defaultAgent.velocity_);

        agent.id = this.agentId++;
        this.aid2agent[agent.id] = agent;
        this.agentIdLst.push(agent.id);

        return agent.id;
    }

    removeAgent(aid: number) {
        if(this.hasAgent(aid)) {
            delete this.aid2agent[aid];
            let idx = this.agentIdLst.indexOf(aid);
            this.agentIdLst[idx] = this.agentIdLst[this.agentIdLst.length - 1];
            this.agentIdLst.length--;
        }
    }

    hasAgent(aid: number) {
        return !!this.aid2agent[aid];
    }

    setAgentMass(agentNo: number, mass: number) {
        this.aid2agent[agentNo].mass = mass;
    }

    getAgentMass(agentNo: number) {
        return this.aid2agent[agentNo].mass;
    }

    setAgentRadius(agentNo: number, radius: number) {
        this.aid2agent[agentNo].radius_ = radius;
    }

    /**
     * 
     * @param neighborDist 在寻找周围邻居的搜索距离，这个值设置过大，会让小球在很远距离时做出避障行为
     * @param maxNeighbors 寻找周围邻居的最大数目，这个值设置越大，最终计算的速度越精确，但会增大计算量
     * @param timeHorizon 代表计算动态的物体时的时间窗口
     * @param timeHorizonObst 代表计算静态的物体时的时间窗口，比如在RTS游戏中，小兵向城墙移动时，没必要做出避障，这个值需要 设置得很小
     * @param radius 代表计算ORCA时的小球的半径，这个值不一定与小球实际显示的半径一样，偏小有利于小球移动顺畅
     * @param maxSpeed 小球最大速度值
     * @param velocity 小球初始速度
     */
    setAgentDefaults(neighborDist: number, maxNeighbors: number, timeHorizon: number, timeHorizonObst: number, radius: number,  maxSpeed: number, velocity: Vector2) {
        if (!this.defaultAgent) {
        	this.defaultAgent = new Agent();
        }

        this.defaultAgent.maxNeighbors_ = maxNeighbors;
        this.defaultAgent.maxSpeed_ = maxSpeed;
        this.defaultAgent.neighborDist = neighborDist;
        this.defaultAgent.radius_ = radius;
        this.defaultAgent.timeHorizon = timeHorizon;
        this.defaultAgent.timeHorizonObst = timeHorizonObst;
        this.defaultAgent.velocity_ = velocity;
    }
    
    run(dt: number) {	
    	this.kdTree.buildAgentTree(this.getNumAgents());
        let agentNum = this.agentIdLst.length;
    	for (let i = 0; i < agentNum; i++) {
	    	this.aid2agent[this.agentIdLst[i]].computeNeighbors(this);
	        this.aid2agent[this.agentIdLst[i]].computeNewVelocity(dt);
        }
        for(let i = 0; i < agentNum; i++) {
            this.aid2agent[this.agentIdLst[i]].update(dt);
        }
    	
    	this.time += dt;
    }
    
	
    addObstacle(vertices: Vector2[]) {
        if (vertices.length < 2) {
            return -1;
        }

        let obstacleNo = this.obstacles.length;

        for (let i = 0; i < vertices.length; ++i) {
            let obstacle = new Obstacle();
            obstacle.point = vertices[i];
            if (i != 0) {
                obstacle.previous = this.obstacles[this.obstacles.length - 1];
                obstacle.previous.next = obstacle;
            }
            if (i == vertices.length - 1) {
                obstacle.next = this.obstacles[obstacleNo];
                obstacle.next.previous = obstacle;
            }
            obstacle.direction = RVOMath.normalize(vertices[(i == vertices.length - 1 ? 0 : i + 1)].minus(vertices[i]));

            if (vertices.length == 2) {
                obstacle.convex = true;
            } 
            else {
                obstacle.convex = (RVOMath.leftOf(vertices[(i == 0 ? vertices.length - 1 : i - 1)], vertices[i], vertices[(i == vertices.length - 1 ? 0 : i + 1)]) >= 0);
            }

            obstacle.id = this.obstacles.length;

            this.obstacles.push(obstacle);
        }

        return obstacleNo;
    }

    processObstacles() {
        this.kdTree.buildObstacleTree();
    };

    queryVisibility(point1: Vector2, point2: Vector2, radius: number) {
        return this.kdTree.queryVisibility(point1, point2, radius);
    };

    getObstacles() {
    	return this.obstacles;
    }

    clear() {
        this.agentIdLst.length = 0;
        this.agentId = 0;
        this.aid2agent = Object.create(null);
        this.defaultAgent = null;
        this.kdTree = new KdTree();
        this.obstacles.length = 0;
    }
}