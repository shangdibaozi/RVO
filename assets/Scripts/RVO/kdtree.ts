import { RVOMath, Obstacle, Vector2 } from "./Common";
import { Simulator } from "./Simulator";
import { Agent } from "./Agent";

class FloatPair {
    a: number;
    b: number;
    constructor(a: number, b: number) {
        this.a = a;
		this.b = b;
    }

    lessThan(rhs: FloatPair) {
        return this.a < rhs.a || !(rhs.a < this.a) && this.b < rhs.b;
    }

    lessEqualThan(rhs: FloatPair) {
        return (this.a == rhs.a && this.b == rhs.b) || this.lessThan(rhs);
    }

    bigThan(rhs: FloatPair) {
        return !this.lessEqualThan(rhs);
    }

    bigEqualThan(rhs: FloatPair) {
        return !this.lessThan(rhs);
    }
}

class AgentTreeNode {
    begin: number;
    end: number;
    left: number;
    right: number;
    maxX: number;
    maxY: number;
    minX: number;
    minY: number;
}

class ObstacleTreeNode {
    obstacle: Obstacle;
    left: ObstacleTreeNode;
    right: ObstacleTreeNode;
}

export class KdTree {
    /**
     * The maximum size of an agent k-D tree leaf.
     */
    MAX_LEAF_SIZE = 10;
    agents: Agent[] = null;
	agentTree: AgentTreeNode[] = [];
	obstacleTree: ObstacleTreeNode = null;
	
	
	buildAgentTree(agentNum: number) {
		if (!this.agents || this.agents.length != agentNum) {
            this.agents = new Array<Agent>(agentNum);
            for(let i = 0; i < this.agents.length; i++) {
                this.agents[i] = Simulator.instance.getAgent(i);
            }
            
            this.agentTree = new Array<AgentTreeNode>(2 * this.agents.length);
			for (let i = 0; i < this.agentTree.length; i++) {
				this.agentTree[i] = new AgentTreeNode();
			}
		}
		
		if (this.agents.length != 0) {
			this.buildAgentTreeRecursive(0, this.agents.length, 0);
		}
    }
    
    buildObstacleTree() {
        this.obstacleTree = new ObstacleTreeNode();
        let obstacles = new Array<Obstacle>(Simulator.instance.obstacles.length);
        for(let i = 0; i < obstacles.length; i++) {
            obstacles[i] = Simulator.instance.obstacles[i];
        }
		this.obstacleTree = this.buildObstacleTreeRecursive(obstacles);
    }
    
    computeAgentNeighbors(agent: Agent, rangeSq: number) {
		return this.queryAgentTreeRecursive(agent, rangeSq, 0);
    }
    
    computeObstacleNeighbors(agent: Agent, rangeSq: number) {
		this.queryObstacleTreeRecursive(agent, rangeSq, this.obstacleTree);
    }
    
    queryVisibility (q1: Vector2, q2: Vector2, radius: number) {
        return this.queryVisibilityRecursive(q1, q2, radius, this.obstacleTree);
    }
    
	buildAgentTreeRecursive(begin: number, end: number, node: number) {
		this.agentTree[node].begin = begin;
		this.agentTree[node].end = end;
		this.agentTree[node].minX = this.agentTree[node].maxX = this.agents[begin].position_.x;
		this.agentTree[node].minY = this.agentTree[node].maxY = this.agents[begin].position_.y;
		
		for (let i = begin + 1; i < end; ++i) {
			this.agentTree[node].maxX = Math.max(this.agentTree[node].maxX, this.agents[i].position_.x);
			this.agentTree[node].minX = Math.min(this.agentTree[node].minX, this.agents[i].position_.x);
			this.agentTree[node].maxY = Math.max(this.agentTree[node].maxY, this.agents[i].position_.y);
			this.agentTree[node].minY = Math.min(this.agentTree[node].minY, this.agents[i].position_.y);
		}
		
		if (end - begin > this.MAX_LEAF_SIZE) {
			// no leaf node
			let isVertical = (this.agentTree[node].maxX - this.agentTree[node].minX) > (this.agentTree[node].maxY - this.agentTree[node].minY);
			let splitValue = 0.5 * (isVertical ? this.agentTree[node].maxX + this.agentTree[node].minX : this.agentTree[node].maxY + this.agentTree[node].minY);
			
			let left = begin;
			let right = end;
			
			while (left < right) {
				while (left < right && (isVertical ? this.agents[left].position_.x : this.agents[left].position_.y) < splitValue) {
                    ++left;
                }

                while (right > left && (isVertical ? this.agents[right - 1].position_.x : this.agents[right - 1].position_.y) >= splitValue) {
                    --right;
                }

                if (left < right) {
                    let tmp = this.agents[left];
                    this.agents[left] = this.agents[right - 1];
                    this.agents[right - 1] = tmp;
                    ++left;
                    --right;
                }
			}
			
			let leftSize = left - begin;
			if (leftSize == 0) {
				++leftSize;
				++left;
				++right;
			}
			
			this.agentTree[node].left = node + 1;
            this.agentTree[node].right = node + 2 * leftSize;

            this.buildAgentTreeRecursive(begin, left, this.agentTree[node].left);
            this.buildAgentTreeRecursive(left, end, this.agentTree[node].right);
		}
    }
    

	buildObstacleTreeRecursive(obstacles: Obstacle[]) {
		if (obstacles.length == 0) {
			return null;
        } 
        else {
			let node = new ObstacleTreeNode();
			let optimalSplit = 0;
            let minLeft = obstacles.length;
            let minRight = minLeft;
			
			for (let i = 0; i < obstacles.length; ++i) {
				let leftSize = 0;
				let rightSize = 0;
				
				let obstacleI1 = obstacles[i];
				let obstacleI2 = obstacleI1.next;
				
				for (let j = 0; j < obstacles.length; j++) {
					if (i == j) {
						continue;
					}
					
					let obstacleJ1 = obstacles[j];
					let obstacleJ2 = obstacleJ1.next;
					
					let j1LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
                    let j2LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);
					
                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
                        ++leftSize;
                    }
                    else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
                        ++rightSize;
                    }
                    else {
                        ++leftSize;
                        ++rightSize;
                    }
                    
                    let fp1 = new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize));
                    let fp2 = new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight));
                    
                    if (fp1.bigEqualThan(fp2)) {
                    	break;
                    }
				}
				
				let fp1 = new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize));
				let fp2 = new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight));
				
				if (fp1.lessThan(fp2)) {
					minLeft = leftSize;
					minRight = rightSize;
					optimalSplit = i;
				}
			}
			
			{
                /* Build split node. */
				let leftObstacles: Obstacle[] = [];
                for (let n = 0; n < minLeft; ++n) leftObstacles.push(null);
                
                let rightObstacles: Obstacle[] = [];
                for (let n = 0; n < minRight; ++n) rightObstacles.push(null);

                let leftCounter = 0;
                let rightCounter = 0;
                let i = optimalSplit;

                let obstacleI1 = obstacles[i];
                let obstacleI2 = obstacleI1.next;

                for (let j = 0; j < obstacles.length; ++j) {
                    if (i == j) {
                        continue;
                    }

                    let obstacleJ1 = obstacles[j];
                    let obstacleJ2 = obstacleJ1.next;

                    let j1LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
                    let j2LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
                        leftObstacles[leftCounter++] = obstacles[j];
                    }
                    else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
                        rightObstacles[rightCounter++] = obstacles[j];
                    }
                    else {
                        /* Split obstacle j. */
                        let t = RVOMath.det(obstacleI2.point.minus(obstacleI1.point), obstacleJ1.point.minus(obstacleI1.point)) / 
                        	RVOMath.det(obstacleI2.point.minus(obstacleI1.point), obstacleJ1.point.minus(obstacleJ2.point));

                        let splitpoint = obstacleJ1.point.plus( (obstacleJ2.point.minus(obstacleJ1.point)).scale(t) );

                        let newObstacle = new Obstacle();
                        newObstacle.point = splitpoint;
                        newObstacle.previous = obstacleJ1;
                        newObstacle.next = obstacleJ2;
                        newObstacle.convex = true;
                        newObstacle.direction = obstacleJ1.direction;

                        newObstacle.id = Simulator.instance.obstacles.length;

                        Simulator.instance.obstacles.push(newObstacle);

                        obstacleJ1.next = newObstacle;
                        obstacleJ2.previous = newObstacle;

                        if (j1LeftOfI > 0.0) {
                            leftObstacles[leftCounter++] = obstacleJ1;
                            rightObstacles[rightCounter++] = newObstacle;
                        }
                        else {
                            rightObstacles[rightCounter++] = obstacleJ1;
                            leftObstacles[leftCounter++] = newObstacle;
                        }
                    }
                }

                node.obstacle = obstacleI1;
                node.left = this.buildObstacleTreeRecursive(leftObstacles);
                node.right = this.buildObstacleTreeRecursive(rightObstacles);
                return node;
            }
		}
	}
	
	
	queryAgentTreeRecursive(agent: Agent, rangeSq: number, node: number) {
		if (this.agentTree[node].end - this.agentTree[node].begin <= this.MAX_LEAF_SIZE) {
            for (let i = this.agentTree[node].begin; i < this.agentTree[node].end; ++i) {
                rangeSq = agent.insertAgentNeighbor(this.agents[i], rangeSq);
            }
        }
        else {
            let distSqLeft = RVOMath.sqr(Math.max(0, this.agentTree[this.agentTree[node].left].minX - agent.position_.x)) + 
	            RVOMath.sqr(Math.max(0, agent.position_.x - this.agentTree[this.agentTree[node].left].maxX)) + 
	            RVOMath.sqr(Math.max(0, this.agentTree[this.agentTree[node].left].minY - agent.position_.y)) + 
	            RVOMath.sqr(Math.max(0, agent.position_.y - this.agentTree[this.agentTree[node].left].maxY));

            let distSqRight = RVOMath.sqr(Math.max(0, this.agentTree[this.agentTree[node].right].minX - agent.position_.x)) +
	            RVOMath.sqr(Math.max(0, agent.position_.x - this.agentTree[this.agentTree[node].right].maxX)) +
	            RVOMath.sqr(Math.max(0, this.agentTree[this.agentTree[node].right].minY - agent.position_.y)) +
	            RVOMath.sqr(Math.max(0, agent.position_.y - this.agentTree[this.agentTree[node].right].maxY));

            if (distSqLeft < distSqRight) {
                if (distSqLeft < rangeSq) {
                    rangeSq = this.queryAgentTreeRecursive(agent, rangeSq, this.agentTree[node].left);

                    if (distSqRight < rangeSq) {
                        rangeSq = this.queryAgentTreeRecursive(agent, rangeSq, this.agentTree[node].right);
                    }
                }
            }
            else {
                if (distSqRight < rangeSq) {
                    rangeSq = this.queryAgentTreeRecursive(agent, rangeSq, this.agentTree[node].right);

                    if (distSqLeft < rangeSq) {
                        rangeSq = this.queryAgentTreeRecursive(agent, rangeSq, this.agentTree[node].left);
                    }
                }
            }

        }
        return rangeSq;
	}
	
	// pass ref range
	queryObstacleTreeRecursive(agent: Agent, rangeSq: number, node: ObstacleTreeNode) {
        if (node == null) {
            return rangeSq;
        }
        else {
            let obstacle1 = node.obstacle;
            let obstacle2 = obstacle1.next;

            let agentLeftOfLine = RVOMath.leftOf(obstacle1.point, obstacle2.point, agent.position_);

            rangeSq = this.queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0 ? node.left : node.right));

            let distSqLine = RVOMath.sqr(agentLeftOfLine) / RVOMath.absSq(obstacle2.point.minus(obstacle1.point));

            if (distSqLine < rangeSq)
            {
                if (agentLeftOfLine < 0)
                {
                    /*
                     * Try obstacle at this node only if is on right side of
                     * obstacle (and can see obstacle).
                     */
                    agent.insertObstacleNeighbor(node.obstacle, rangeSq);
                }

                /* Try other side of line. */
                this.queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0 ? node.right : node.left));
            }
            return rangeSq;
        }
    }

    

    queryVisibilityRecursive(q1: Vector2, q2: Vector2, radius: number, node: ObstacleTreeNode) {
        if (node == null) {
            return true;
        }
        else {
            let obstacle1 = node.obstacle;
            let obstacle2 = obstacle1.next;

            let q1LeftOfI = RVOMath.leftOf(obstacle1.point, obstacle2.point, q1);
            let q2LeftOfI = RVOMath.leftOf(obstacle1.point, obstacle2.point, q2);
            let invLengthI = 1.0 / RVOMath.absSq(obstacle2.point.minus(obstacle1.point));

            if (q1LeftOfI >= 0 && q2LeftOfI >= 0)
            {
                return this.queryVisibilityRecursive(q1, q2, radius, node.left) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || this.queryVisibilityRecursive(q1, q2, radius, node.right));
            }
            else if (q1LeftOfI <= 0 && q2LeftOfI <= 0)
            {
                return this.queryVisibilityRecursive(q1, q2, radius, node.right) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || this.queryVisibilityRecursive(q1, q2, radius, node.left));
            }
            else if (q1LeftOfI >= 0 && q2LeftOfI <= 0)
            {
                /* One can see through obstacle from left to right. */
                return this.queryVisibilityRecursive(q1, q2, radius, node.left) && this.queryVisibilityRecursive(q1, q2, radius, node.right);
            }
            else
            {
                let point1LeftOfQ = RVOMath.leftOf(q1, q2, obstacle1.point);
                let point2LeftOfQ = RVOMath.leftOf(q1, q2, obstacle2.point);
                let invLengthQ = 1.0 / RVOMath.absSq(q2.minus(q1));

                return (point1LeftOfQ * point2LeftOfQ >= 0 && RVOMath.sqr(point1LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && RVOMath.sqr(point2LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && this.queryVisibilityRecursive(q1, q2, radius, node.left) && this.queryVisibilityRecursive(q1, q2, radius, node.right));
            }
        }
    }
}        