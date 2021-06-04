import { RVOMath, Vector2, Line, KeyValuePair, Obstacle } from "./Common";
import { Simulator } from "./Simulator";


export class Agent {
	agentNeighbors_: KeyValuePair<number, Agent>[] = [];
	obstaclNeighbors_: KeyValuePair<number, Obstacle>[] = [];
	orcaLines_: Line[] = [];
    position_: Vector2 = new Vector2(0, 0);
	prefVelocity_: Vector2 = new Vector2(0, 0);
	velocity_: Vector2 = new Vector2(0, 0);
	id: number = 0;
	maxNeighbors_: number = 0;
	maxSpeed_: number = 0.0;
	neighborDist: number = 0.0;
	radius_: number = 0.0;
	timeHorizon: number = 0.0;
	timeHorizonObst: number = 0.0;
	newVelocity_: Vector2 = new Vector2(0, 0);
    mass: number = 1;
    
    
    computeNeighbors(sim: Simulator) {
        this.obstaclNeighbors_.length = 0;
        let rangeSq = (this.timeHorizonObst * this.maxSpeed_ + this.radius_) ** 2;
        sim.kdTree.computeObstacleNeighbors(this, rangeSq);

        this.agentNeighbors_.length = 0;

        if (this.maxNeighbors_ > 0) {
            rangeSq = this.neighborDist ** 2;
            rangeSq = sim.kdTree.computeAgentNeighbors(this, rangeSq);
        }
    }

    /* Search for the best new velocity. */
    computeNewVelocity(dt: number) {	
        this.orcaLines_.length = 0;
        let orcaLines = this.orcaLines_;

        let invTimeHorizonObst = 1.0 / this.timeHorizonObst;

        /* Create obstacle ORCA lines. */
        for (let i = 0; i < this.obstaclNeighbors_.length; ++i) {
            let obstacle1 = this.obstaclNeighbors_[i].value;
            let obstacle2 = obstacle1.next;

            let relativePosition1 = obstacle1.point.minus(this.position_);
            let relativePosition2 = obstacle2.point.minus(this.position_);

            /* 
             * Check if velocity obstacle of obstacle is already taken care of by
             * previously constructed obstacle ORCA lines.
             */
            let alreadyCovered = false;

            for (let j = 0; j < orcaLines.length; ++j) {
                if (RVOMath.det(relativePosition1.scale(invTimeHorizonObst).minus(orcaLines[j].point), orcaLines[j].direction) - invTimeHorizonObst * this.radius_ >= -RVOMath.RVO_EPSILON 
                    && RVOMath.det(relativePosition2.scale(invTimeHorizonObst).minus(orcaLines[j].point), orcaLines[j].direction) - invTimeHorizonObst * this.radius_ >= -RVOMath.RVO_EPSILON) {

                    alreadyCovered = true;
                    break;
                }
            }

            if (alreadyCovered) {
                continue;
            }

            /* Not yet covered. Check for collisions. */

            let distSq1 = RVOMath.absSq(relativePosition1);
            let distSq2 = RVOMath.absSq(relativePosition2);

            let radiusSq = RVOMath.sqr(this.radius_);

            let obstacleVector = obstacle2.point.minus(obstacle1.point);
            let s = relativePosition1.scale(-1).multiply(obstacleVector) / RVOMath.absSq(obstacleVector);
            let distSqLine = RVOMath.absSq(relativePosition1.scale(-1).minus(obstacleVector.scale(s))); 

            let line = new Line();
            if (s < 0 && distSq1 <= radiusSq) {
                /* Collision with left vertex. Ignore if non-convex. */
                if (obstacle1.convex) {
                    line.point = new Vector2(0, 0);
                    line.direction = RVOMath.normalize(new Vector2(-relativePosition1.y, relativePosition1.x));
                    orcaLines.push(line);
                }
                continue;
            }
            else if (s > 1 && distSq2 <= radiusSq) {
                /* Collision with right vertex. Ignore if non-convex 
                 * or if it will be taken care of by neighoring obstace */
                if (obstacle2.convex && RVOMath.det(relativePosition2, obstacle2.direction) >= 0) {
                    line.point = new Vector2(0, 0);
                    line.direction = RVOMath.normalize(new Vector2(-relativePosition2.y, relativePosition2.x));
                    orcaLines.push(line);
                }
                continue;
            }
            else if (s >= 0 && s <= 1 && distSqLine <= radiusSq) {
                /* Collision with obstacle segment. */
                line.point = new Vector2(0, 0);
                line.direction = obstacle1.direction.scale(-1);
                orcaLines.push(line);
                continue;
            }

            /* 
             * No collision.  
             * Compute legs. When obliquely viewed, both legs can come from a single
             * vertex. Legs extend cut-off line when nonconvex vertex.
             */
            let leftLegDirection: Vector2, rightLegDirection: Vector2;

            if (s < 0 && distSqLine <= radiusSq) {
                /*
                 * Obstacle viewed obliquely so that left vertex
                 * defines velocity obstacle.
                 */
                if (!obstacle1.convex) {
                    /* Ignore obstacle. */
                    continue;
                }

                obstacle2 = obstacle1;

                let leg1 = Math.sqrt(distSq1 - radiusSq);
                leftLegDirection = (new Vector2(relativePosition1.x * leg1 - relativePosition1.y * this.radius_, relativePosition1.x * this.radius_ + relativePosition1.y * leg1)).scale(1 / distSq1);
                rightLegDirection = (new Vector2(relativePosition1.x * leg1 + relativePosition1.y * this.radius_, -relativePosition1.x * this.radius_ + relativePosition1.y * leg1)).scale(1 / distSq1);
            }
            else if (s > 1 && distSqLine <= radiusSq) {
                /*
                 * Obstacle viewed obliquely so that
                 * right vertex defines velocity obstacle.
                 */
                if (!obstacle2.convex) {
                    /* Ignore obstacle. */
                    continue;
                }

                obstacle1 = obstacle2;

                let leg2 = Math.sqrt(distSq2 - radiusSq);
                leftLegDirection = (new Vector2(relativePosition2.x * leg2 - relativePosition2.y * this.radius_, relativePosition2.x * this.radius_ + relativePosition2.y * leg2)).scale(1 / distSq2);
                rightLegDirection = (new Vector2(relativePosition2.x * leg2 + relativePosition2.y * this.radius_, -relativePosition2.x * this.radius_ + relativePosition2.y * leg2)).scale(1 / distSq2);
            }
            else {
                /* Usual situation. */
                if (obstacle1.convex) {
                    let leg1 = Math.sqrt(distSq1 - radiusSq);
                    leftLegDirection = (new Vector2(relativePosition1.x * leg1 - relativePosition1.y * this.radius_, relativePosition1.x * this.radius_ + relativePosition1.y * leg1)).scale(1 / distSq1);
                }
                else {
                    /* Left vertex non-convex; left leg extends cut-off line. */
                    leftLegDirection = obstacle1.direction.scale(-1);
                }

                if (obstacle2.convex) {
                    let leg2 = Math.sqrt(distSq2 - radiusSq);
                    rightLegDirection = (new Vector2(relativePosition2.x * leg2 + relativePosition2.y * this.radius_, -relativePosition2.x * this.radius_ + relativePosition2.y * leg2)).scale(1 / distSq2);
                }
                else {
                    /* Right vertex non-convex; right leg extends cut-off line. */
                    rightLegDirection = obstacle1.direction;
                }
            }

            /* 
             * Legs can never point into neighboring edge when convex vertex,
             * take cutoff-line of neighboring edge instead. If velocity projected on
             * "foreign" leg, no constraint is added. 
             */

            let leftNeighbor = obstacle1.previous;

            let isLeftLegForeign = false;
            let isRightLegForeign = false;

            if (obstacle1.convex && RVOMath.det(leftLegDirection, leftNeighbor.direction.scale(-1)) >= 0.0) {
                /* Left leg points into obstacle. */
                leftLegDirection = leftNeighbor.direction.scale(-1);
                isLeftLegForeign = true;
            }

            if (obstacle2.convex && RVOMath.det(rightLegDirection, obstacle2.direction) <= 0.0) {
                /* Right leg points into obstacle. */
                rightLegDirection = obstacle2.direction;
                isRightLegForeign = true;
            }

            /* Compute cut-off centers. */
            let leftCutoff = obstacle1.point.minus(this.position_).scale(invTimeHorizonObst);
            let rightCutoff = obstacle2.point.minus(this.position_).scale(invTimeHorizonObst);
            let cutoffVec = rightCutoff.minus(leftCutoff);

            /* Project current velocity on velocity obstacle. */

            /* Check if current velocity is projected on cutoff circles. */
            let t = (obstacle1 == obstacle2) ? 0.5 : this.velocity_.minus(leftCutoff).multiply(cutoffVec) / RVOMath.absSq(cutoffVec);
            let tLeft = this.velocity_.minus(leftCutoff).multiply(leftLegDirection);
            let tRight = this.velocity_.minus(rightCutoff).multiply(rightLegDirection);

            if ((t < 0.0 && tLeft < 0.0) || (obstacle1 == obstacle2 && tLeft < 0.0 && tRight < 0.0)) {
                /* Project on left cut-off circle. */
                let unitW = RVOMath.normalize(this.velocity_.minus(leftCutoff));

                line.direction = new Vector2(unitW.y, -unitW.x);
                line.point = leftCutoff.plus(unitW.scale(this.radius_ * invTimeHorizonObst));
                orcaLines.push(line);
                continue;
            }
            else if (t > 1.0 && tRight < 0.0) {
                /* Project on right cut-off circle. */
                let unitW = RVOMath.normalize(this.velocity_.minus(rightCutoff));

                line.direction = new Vector2(unitW.y, -unitW.x);
                line.point = rightCutoff.plus(unitW.scale(this.radius_ * invTimeHorizonObst));
                orcaLines.push(line);
                continue;
            }

            /* 
             * Project on left leg, right leg, or cut-off line, whichever is closest
             * to velocity.
             */
            let distSqCutoff = ((t < 0.0 || t > 1.0 || obstacle1 == obstacle2) ? Infinity : RVOMath.absSq(this.velocity_.minus(cutoffVec.scale(t).plus(leftCutoff))));
            let distSqLeft = ((tLeft < 0.0) ? Infinity : RVOMath.absSq(this.velocity_.minus(leftLegDirection.scale(tLeft).plus(leftCutoff))));
            let distSqRight = ((tRight < 0.0) ? Infinity : RVOMath.absSq(this.velocity_.minus(rightLegDirection.scale(tRight).plus(rightCutoff))));

            if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
                /* Project on cut-off line. */
                line.direction = obstacle1.direction.scale(-1);
                let aux = new Vector2(-line.direction.y, line.direction.x);
                line.point = aux.scale(this.radius_ * invTimeHorizonObst).plus(leftCutoff); 
                orcaLines.push(line);
                continue;
            }
            else if (distSqLeft <= distSqRight) {
                /* Project on left leg. */
                if (isLeftLegForeign) {
                    continue;
                }

                line.direction = leftLegDirection;
                let aux = new Vector2(-line.direction.y, line.direction.x);
                line.point = aux.scale(this.radius_ * invTimeHorizonObst).plus(leftCutoff);
                orcaLines.push(line);
                continue;
            }
            else {
                /* Project on right leg. */
                if (isRightLegForeign) {
                    continue;
                }

                line.direction = rightLegDirection.scale(-1);
                let aux = new Vector2(-line.direction.y, line.direction.x);
                line.point = aux.scale(this.radius_ * invTimeHorizonObst).plus(rightCutoff);
                orcaLines.push(line);
                continue;
            }
        }

        let numObstLines = orcaLines.length;

        let invTimeHorizon = 1.0 / this.timeHorizon;

        /* Create agent ORCA lines. */
        for (let i = 0; i < this.agentNeighbors_.length; ++i) {
            let other = this.agentNeighbors_[i].value;

            let relativePosition = other.position_.minus(this.position_);

            // mass
            let massRatio = (other.mass / (this.mass + other.mass));
            let neighborMassRatio = (this.mass / (this.mass + other.mass));

            let velocityOpt = (massRatio >= 0.5 ?  (this.velocity_.minus(this.velocity_.scale(massRatio)).scale(2)) : this.prefVelocity_.plus(this.velocity_.minus(this.prefVelocity_).scale(massRatio * 2)));
            let neighborVelocityOpt = (neighborMassRatio >= 0.5 ? other.velocity_.scale(2).scale(1 - neighborMassRatio) : (other.prefVelocity_.plus(other.velocity_.minus(other.prefVelocity_).scale(2 * neighborMassRatio))));

            let relativeVelocity = velocityOpt.minus(neighborVelocityOpt);//this.velocity.minus(other.velocity);
            let distSq = RVOMath.absSq(relativePosition);
            let combinedRadius = this.radius_ + other.radius_;
            let combinedRadiusSq = RVOMath.sqr(combinedRadius);

            let line = new Line();
            let u: Vector2;

            if (distSq > combinedRadiusSq) {
                /* No collision. */
                let w = relativeVelocity.minus(relativePosition.scale(invTimeHorizon)); // Vector
                /* Vector from cutoff center to relative velocity. */
                let wLengthSq = RVOMath.absSq(w);

                let dotProduct1 = w.multiply(relativePosition);

                if (dotProduct1 < 0.0 && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
                    /* Project on cut-off circle. */
                    let wLength = Math.sqrt(wLengthSq);
                    let unitW = w.scale(1 / wLength);

                    line.direction = new Vector2(unitW.y, -unitW.x);
                    u = unitW.scale(combinedRadius * invTimeHorizon - wLength);
                }
                else {
                    /* Project on legs. */
                    let leg = Math.sqrt(distSq - combinedRadiusSq);

                    if (RVOMath.det(relativePosition, w) > 0.0) {
                        /* Project on left leg. */
                    	let aux = new Vector2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg);
                        line.direction = aux.scale(1 / distSq);
                    }
                    else {
                        /* Project on right leg. */
                    	let aux = new Vector2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg);
                        line.direction = aux.scale(-1 / distSq);
                    }

                    let dotProduct2 = relativeVelocity.multiply(line.direction);
                    u = line.direction.scale(dotProduct2).minus(relativeVelocity);
                }
            }
            else {
                /* Collision. Project on cut-off circle of time timeStep. */
                let invTimeStep = 1.0 / dt;

                /* Vector from cutoff center to relative velocity. */
                let w = relativeVelocity.minus(relativePosition.scale(invTimeStep));

                let wLength = RVOMath.abs(w);
                let unitW = w.scale(1 / wLength);

                line.direction = new Vector2(unitW.y, -unitW.x);
                u = unitW.scale(combinedRadius * invTimeStep - wLength);
            }

        
            // line.point = u.scale(0.5).plus(this.velocity);
            line.point = velocityOpt.plus(u.scale(massRatio));
            orcaLines.push(line);
        }

        let lineFail = this.linearProgram2(orcaLines, this.maxSpeed_, this.prefVelocity_, false, this.newVelocity_);

        if (lineFail < orcaLines.length) {
            this.linearProgram3(orcaLines, numObstLines, lineFail, this.maxSpeed_, this.newVelocity_);
        }
    }

    insertAgentNeighbor(agent: Agent, rangeSq: number) {
        if (this != agent) {
            let distSq = RVOMath.absSq(this.position_.minus(agent.position_));

            if (distSq < rangeSq) {
                if (this.agentNeighbors_.length < this.maxNeighbors_) {
                    this.agentNeighbors_.push(new KeyValuePair(distSq, agent));
                }
                let i = this.agentNeighbors_.length - 1;
                while (i != 0 && distSq < this.agentNeighbors_[i - 1].key) {
                    this.agentNeighbors_[i] = this.agentNeighbors_[i - 1];
                    --i;
                }
                this.agentNeighbors_[i] = new KeyValuePair<number, Agent>(distSq, agent);

                if (this.agentNeighbors_.length == this.maxNeighbors_) {
                    rangeSq = this.agentNeighbors_[this.agentNeighbors_.length - 1].key;
                }
            }
        }
        return rangeSq;
    }

    insertObstacleNeighbor(obstacle: Obstacle, rangeSq: number) {
        let nextObstacle = obstacle.next;

        let distSq = RVOMath.distSqPointLineSegment(obstacle.point, nextObstacle.point, this.position_);

        if (distSq < rangeSq) {
            this.obstaclNeighbors_.push(new KeyValuePair<number, Obstacle>(distSq, obstacle));

            let i = this.obstaclNeighbors_.length - 1;
            while (i != 0 && distSq < this.obstaclNeighbors_[i - 1].key) {
                this.obstaclNeighbors_[i] = this.obstaclNeighbors_[i - 1];
                --i;
            }
            this.obstaclNeighbors_[i] = new KeyValuePair<number, Obstacle>(distSq, obstacle);
        }
    }

    update(dt: number) {
        this.velocity_.copy(this.newVelocity_);
        this.position_.copy(this.position_.plus(this.velocity_.scale(dt)));
    };

    linearProgram1(lines: Line[], lineNo: number, radius: number, optVelocity: Vector2, directionOpt: boolean, result: Vector2) {
        let dotProduct = lines[lineNo].point.multiply(lines[lineNo].direction);
        let discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - RVOMath.absSq(lines[lineNo].point);

        if (discriminant < 0.0) {
            /* Max speed circle fully invalidates line lineNo. */
            return false;
        }

        let sqrtDiscriminant = Math.sqrt(discriminant);
        let tLeft = -dotProduct - sqrtDiscriminant;
        let tRight = -dotProduct + sqrtDiscriminant;

        for (let i = 0; i < lineNo; ++i) {
            let denominator = RVOMath.det(lines[lineNo].direction, lines[i].direction);
            let numerator = RVOMath.det(lines[i].direction, lines[lineNo].point.minus(lines[i].point));

            if (Math.abs(denominator) <= RVOMath.RVO_EPSILON) {
                /* Lines lineNo and i are (almost) parallel. */
                if (numerator < 0.0) {
                    return false;
                }
                else {
                    continue;
                }
            }

            let t = numerator / denominator;

            if (denominator >= 0.0) {
                /* Line i bounds line lineNo on the right. */
                tRight = Math.min(tRight, t);
            }
            else {
                /* Line i bounds line lineNo on the left. */
                tLeft = Math.max(tLeft, t);
            }

            if (tLeft > tRight) {
                return false;
            }
        }

        if (directionOpt) {
            if (optVelocity.multiply(lines[lineNo].direction) > 0.0) {
                // Take right extreme
            	result.copy(lines[lineNo].point.plus(lines[lineNo].direction.scale(tRight)));
            }
            else {
                // Take left extreme.
            	result.copy(lines[lineNo].point.plus(lines[lineNo].direction.scale(tLeft)));
            }
        }
        else {
            // Optimize closest point
            let t = lines[lineNo].direction.multiply(optVelocity.minus(lines[lineNo].point));
            if (t < tLeft) {
            	result.copy(lines[lineNo].point.plus(lines[lineNo].direction.scale(tLeft)));
            }
            else if (t > tRight) {
            	result.copy(lines[lineNo].point.plus(lines[lineNo].direction.scale(tRight)));
            }
            else {
            	result.copy(lines[lineNo].point.plus(lines[lineNo].direction.scale(t)));
            }
        }
        
        return true;
    }
    
    linearProgram2(lines: Line[], radius: number, optVelocity: Vector2, directionOpt: boolean, result: Vector2) {
        // directionOpt 第一次为false，第二次为true，directionOpt主要用在 linearProgram1 里面
        if (directionOpt) {
            /* 
             * Optimize direction. Note that the optimization velocity is of unit
             * length in this case.
             */
        	result.copy(optVelocity.scale(radius));
        }
        else if (RVOMath.absSq(optVelocity) > RVOMath.sqr(radius)) {
            /* Optimize closest point and outside circle. */
        	result.copy(RVOMath.normalize(optVelocity).scale(radius));
        }
        else {
            /* Optimize closest point and inside circle. */
        	result.copy(optVelocity);
        }

        for (let i = 0; i < lines.length; ++i) {
            if (RVOMath.det(lines[i].direction, lines[i].point.minus(result)) > 0.0) {
                /* Result does not satisfy constraint i. Compute new optimal result. */
                let tempResult = result.clone();
                if (!this.linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
                    result.copy(tempResult);
                    return i;
                }
            }
        }

        return lines.length;
    }

    linearProgram3(lines: Line[], numObstLines: number, beginLine: number, radius: number, result: Vector2) {
        let distance = 0.0;
        // 遍历所有剩余ORCA线
        for (let i = beginLine; i < lines.length; ++i) {
            // 每一条 ORCA 线都需要精确的做出处理，distance 为 最大违规的速度
            if (RVOMath.det(lines[i].direction, lines[i].point.minus(result)) > distance) {
                /* Result does not satisfy constraint of line i. */
                //std::vector<Line> projLines(lines.begin(), lines.begin() + numObstLines);
                let projLines = []; // new List<Line>();
                // 1.静态阻挡的orca线直接加到projLines中
                for (let ii = 0; ii < numObstLines; ++ii) {
                    projLines.push(lines[ii]);
                }
                // 2.动态阻挡的orca线需要重新计算line，从第一个非静态阻挡到当前的orca线
                for (let j = numObstLines; j < i; ++j) {
                    let line = new Line();

                    let determinant = RVOMath.det(lines[i].direction, lines[j].direction);

                    if (Math.abs(determinant) <= RVOMath.RVO_EPSILON) {
                        /* Line i and line j are parallel. */
                        if (lines[i].direction.multiply(lines[j].direction) > 0.0) {
                            /* Line i and line j point in the same direction. */
                            continue;
                        }
                        else {
                            /* Line i and line j point in opposite direction. */
                            line.point = lines[i].point.plus(lines[j].point).scale(0.5);
                        }
                    }
                    else {
                        line.point = lines[i].point.plus(lines[i].direction.scale(RVOMath.det(lines[j].direction, lines[i].point.minus(lines[j].point)) / determinant));
                    }

                    line.direction = RVOMath.normalize(lines[j].direction.minus(lines[i].direction));
                    projLines.push(line);
                }

                let tempResult = result.clone();
                if (this.linearProgram2(projLines, radius, new Vector2(-lines[i].direction.y, lines[i].direction.x), true, result) < projLines.length) {
                    /* This should in principle not happen.  The result is by definition
                     * already in the feasible region of this linear program. If it fails,
                     * it is due to small floating point error, and the current result is
                     * kept.
                     */
                    result.copy(tempResult);
                }

                distance = RVOMath.det(lines[i].direction, lines[i].point.minus(result));
            }
        }
    }
}