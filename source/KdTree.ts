import {Simulator} from "./Simulator";
import {Agent} from "./Agent";
import {AgentTreeNode} from "./AgentTreeNode";
import {Vector2} from "./Vector2";
import {RVOMath} from "./RVOMath";
import {Obstacle} from "./Obstacle";
import {FloatPair} from "./FloatPair";
import {ObstacleTreeNode} from "./ObstacleTreeNode";

export class KdTree {

    private MAX_LEAF_SIZE: number = 10;

    private agents_: Agent[] = [];
    private agentTree_: AgentTreeNode[] = [];
    private obstacleTree_: ObstacleTreeNode;

    constructor() {
    }

    buildAgentTree() {
        //    this.agents_ = new Agent[Simulator.Instance.agents_.length];
        for (let i = 0; i < Simulator.Instance.agents_.length; ++i) {
            this.agents_[i] = Simulator.Instance.agents_[i];
        }

        // this.agentTree_ = new AgentTreeNode[2 *this. agents_.length];
        for (let i = 0; i < this.agents_.length * 2; ++i) {
            this.agentTree_[i] = new AgentTreeNode();

        }

        if (this.agents_.length != 0) {
            this.buildAgentTreeRecursive(0, this.agents_.length, 0);
        }
    }


    buildAgentTreeRecursive(begin: number, end: number, node: number) {
        this.agentTree_[node].begin = begin;
        this.agentTree_[node].end = end;
        this.agentTree_[node].minX = this.agentTree_[node].maxX = this.agents_[begin].position_.x;
        this.agentTree_[node].minY = this.agentTree_[node].maxY = this.agents_[begin].position_.y;

        for (let i = begin + 1; i < end; ++i) {
            this.agentTree_[node].maxX = Math.max(this.agentTree_[node].maxX, this.agents_[i].position_.x);
            this.agentTree_[node].minX = Math.min(this.agentTree_[node].minX, this.agents_[i].position_.x);
            this.agentTree_[node].maxY = Math.max(this.agentTree_[node].maxY, this.agents_[i].position_.y);
            this.agentTree_[node].minY = Math.min(this.agentTree_[node].minY, this.agents_[i].position_.y);
        }

        if (end - begin > this.MAX_LEAF_SIZE) {

            let isVertical: boolean = (this.agentTree_[node].maxX - this.agentTree_[node].minX > this.agentTree_[node].maxY - this.agentTree_[node].minY);
            let splitValue: number = (isVertical ? 0.5 * (this.agentTree_[node].maxX + this.agentTree_[node].minX) : 0.5 * (this.agentTree_[node].maxY + this.agentTree_[node].minY));

            let left: number = begin;
            let right: number = end;

            while (left < right) {
                while (left < right && (isVertical ? this.agents_[left].position_.x : this.agents_[left].position_.y) < splitValue) {
                    ++left;
                }

                while (right > left && (isVertical ? this.agents_[right - 1].position_.x : this.agents_[right - 1].position_.y) >= splitValue) {
                    --right;
                }

                if (left < right) {
                    // std::swap in c++ to JS
                    let tmp: Agent = this.agents_[left];
                    this.agents_[left] = this.agents_[right - 1];
                    this.agents_[right - 1] = tmp;
                    ++left;
                    --right;
                }
            }

            //   var leftSize: number = left - begin;

            if (left == begin) {
                ++left;
                ++right;
            }

            this.agentTree_[node].left = node + 1;
            this.agentTree_[node].right = node + 2 * (left - begin);

            this.buildAgentTreeRecursive(begin, left, this.agentTree_[node].left);
            this.buildAgentTreeRecursive(left, end, this.agentTree_[node].right);
        }
    }


    computeAgentNeighbors(agent: Agent, rangeSq: number) {

        this.queryAgentTreeRecursive(agent, rangeSq, 0);
    }


    queryAgentTreeRecursive(agent: Agent, rangeSq: number, node: number) {


        if (this.agentTree_[node].end - this.agentTree_[node].begin <= this.MAX_LEAF_SIZE) {
            for (let i = this.agentTree_[node].begin; i < this.agentTree_[node].end; ++i) {


                agent.insertAgentNeighbor(this.agents_[i], Agent.rangeSq);
            }
        } else {
            let distSqLeft = RVOMath.sqr(Math.max(0, this.agentTree_[this.agentTree_[node].left].minX - agent.position_.x)) + RVOMath.sqr(Math.max(0, agent.position_.x - this.agentTree_[this.agentTree_[node].left].maxX)) + RVOMath.sqr(Math.max(0, this.agentTree_[this.agentTree_[node].left].minY - agent.position_.y)) + RVOMath.sqr(Math.max(0, agent.position_.y - this.agentTree_[this.agentTree_[node].left].maxY));

            let distSqRight = RVOMath.sqr(Math.max(0, this.agentTree_[this.agentTree_[node].right].minX - agent.position_.x)) + RVOMath.sqr(Math.max(0, agent.position_.x - this.agentTree_[this.agentTree_[node].right].maxX)) + RVOMath.sqr(Math.max(0, this.agentTree_[this.agentTree_[node].right].minY - agent.position_.y)) + RVOMath.sqr(Math.max(0, agent.position_.y - this.agentTree_[this.agentTree_[node].right].maxY));
            if (distSqLeft < distSqRight) {
                if (distSqLeft < Agent.rangeSq) {
                    this.queryAgentTreeRecursive(agent, Agent.rangeSq, this.agentTree_[node].left);

                    if (distSqRight < Agent.rangeSq) {
                        this.queryAgentTreeRecursive(agent, Agent.rangeSq, this.agentTree_[node].right);
                    }
                }
            } else {
                if (distSqRight < Agent.rangeSq) {
                    this.queryAgentTreeRecursive(agent, Agent.rangeSq, this.agentTree_[node].right);

                    if (distSqLeft < Agent.rangeSq) {
                        this.queryAgentTreeRecursive(agent, Agent.rangeSq, this.agentTree_[node].left);
                    }
                }
            }

        }

    }

    buildObstacleTree() {

        this.obstacleTree_ = new ObstacleTreeNode();

        let obstacles: Obstacle[] = [];

        for (let i = 0; i < Simulator.Instance.obstacles_.length; ++i) {
            obstacles[i] = Simulator.Instance.obstacles_[i];

        }
        this.obstacleTree_ = this.buildObstacleTreeRecursive(obstacles);
    }

    buildObstacleTreeRecursive(obstacles: Obstacle[]): ObstacleTreeNode {
        // console.log("buildObstacleTreeRecursive");
        if (obstacles.length == 0) {
            return null;
        }


        let node: ObstacleTreeNode = new ObstacleTreeNode();

        let optimalSplit: number = 0;
        let minLeft: number = obstacles.length;
        let minRight: number = obstacles.length;

        for (let i = 0; i < obstacles.length; ++i) {
            let leftSize: number = 0;
            let rightSize: number = 0;

            let obstacleI1: Obstacle = obstacles[i];
            let obstacleI2: Obstacle = obstacleI1.nextObstacle_;

            /* Compute optimal split node. */
            for (let j = 0; j < obstacles.length; ++j) {
                if (i == j) {
                    continue;
                }

                let obstacleJ1: Obstacle = obstacles[j];
                let obstacleJ2: Obstacle = obstacleJ1.nextObstacle_;

                let j1LeftOfI: number = Vector2.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
                let j2LeftOfI: number = Vector2.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

                if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
                    ++leftSize;
                } else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
                    ++rightSize;
                } else {
                    ++leftSize;
                    ++rightSize;
                }

                let l: FloatPair = new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize));
                let r: FloatPair = new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight));

                if (FloatPair.sup_equal(l, r)) {

                    break;
                }
            }

            let l: FloatPair = new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize));
            let r: FloatPair = new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight));
            if (FloatPair.inf(l, r)) {

                minLeft = leftSize;
                minRight = rightSize;
                optimalSplit = i;
            }
        }


        /* Build split node. */
        let leftObstacles: Obstacle[] = [];
        for (let n = 0; n < minLeft; ++n) leftObstacles[n] = null;
        let rightObstacles: Obstacle[] = [];
        for (let n = 0; n < minRight; ++n) rightObstacles[n] = null;

        let leftCounter: number = 0;
        let rightCounter: number = 0;
        let i: number = optimalSplit;

        let obstacleI1: Obstacle = obstacles[i];
        let obstacleI2: Obstacle = obstacleI1.nextObstacle_;


        for (let j = 0; j < obstacles.length; ++j) {
            if (i == j) {
                continue;
            }

            let obstacleJ1: Obstacle = obstacles[j];
            let obstacleJ2: Obstacle = obstacleJ1.nextObstacle_;

            let j1LeftOfI: number = Vector2.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
            let j2LeftOfI: number = Vector2.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

            if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
                leftObstacles[leftCounter++] = obstacles[j];
            } else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
                rightObstacles[rightCounter++] = obstacles[j];
            } else {
                /* Split obstacle j. */
                //  float t = RVOMath.det(obstacleI2.point_ - obstacleI1.point_, obstacleJ1.point_ - obstacleI1.point_) / RVOMath.det(obstacleI2.point_ - obstacleI1.point_, obstacleJ1.point_ - obstacleJ2.point_);

                let t1: number = Vector2.det(obstacleI2.point_.moins(obstacleI1.point_), obstacleJ1.point_.moins(obstacleI1.point_))
                let t2: number = Vector2.det(obstacleI2.point_.moins(obstacleI1.point_), obstacleJ1.point_.moins(obstacleJ2.point_))
                let t: number = t1 / t2;

                //Vector2 splitpoint = obstacleJ1.point_ + t * (obstacleJ2.point_ - obstacleJ1.point_);
                let vMinus: Vector2 = obstacleJ2.point_.moins(obstacleJ1.point_);
                let vMul: Vector2 = vMinus.mul_k(t);

                let splitpoint: Vector2 = obstacleJ1.point_.plus(vMul);


                let newObstacle: Obstacle = new Obstacle();
                newObstacle.point_ = splitpoint;
                newObstacle.prevObstacle_ = obstacleJ1;
                newObstacle.nextObstacle_ = obstacleJ2;
                newObstacle.isConvex_ = true;
                newObstacle.unitDir_ = obstacleJ1.unitDir_;

                newObstacle.id_ = Simulator.Instance.obstacles_.length;

                Simulator.Instance.obstacles_.push(newObstacle);

                obstacleJ1.nextObstacle_ = newObstacle;
                obstacleJ2.prevObstacle_ = newObstacle;

                if (j1LeftOfI > 0) {
                    leftObstacles[leftCounter++] = obstacleJ1;
                    rightObstacles[rightCounter++] = newObstacle;
                } else {
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


    computeObstacleNeighbors(agent: Agent, rangeSq: number) {
        // console.log("compute"); 
        this.queryObstacleTreeRecursive(agent, rangeSq, this.obstacleTree_);
    }

    queryObstacleTreeRecursive(agent: Agent, rangeSq: number, node: ObstacleTreeNode) {
        if (node == null) {
            return;
        }

        let obstacle1: Obstacle = node.obstacle;
        let obstacle2: Obstacle = obstacle1.nextObstacle_;

        let agentLeftOfLine: number = Vector2.leftOf(obstacle1.point_, obstacle2.point_, agent.position_);

        this.queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0 ? node.left : node.right));

        let distSqLine: number = RVOMath.sqr(agentLeftOfLine) / Vector2.absSq(obstacle2.point_.moins(obstacle1.point_));

        if (distSqLine < rangeSq) {
            if (agentLeftOfLine < 0) {
                /*
                 * Try obstacle at this node only if agent is on right side of
                 * obstacle (and can see obstacle).
                 */
                agent.insertObstacleNeighbor(node.obstacle, rangeSq);

            }

            /* Try other side of line. */
            this.queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0 ? node.right : node.left));

        }
    }


    queryVisibility(q1: Vector2, q2: Vector2, radius: number): boolean {
        return this.queryVisibilityRecursive(q1, q2, radius, this.obstacleTree_);
    }

    queryVisibilityRecursive(q1: Vector2, q2: Vector2, radius: number, node: ObstacleTreeNode): boolean {
        if (node == null) {
            return true;
        } else {
            let obstacle1: Obstacle = node.obstacle;
            let obstacle2: Obstacle = obstacle1.nextObstacle_;

            let q1LeftOfI: number = Vector2.leftOf(obstacle1.point_, obstacle2.point_, q1);
            let q2LeftOfI: number = Vector2.leftOf(obstacle1.point_, obstacle2.point_, q2);
            let invLengthI: number = 1 / Vector2.absSq(obstacle2.point_.moins(obstacle1.point_));

            if (q1LeftOfI >= 0 && q2LeftOfI >= 0) {
                return this.queryVisibilityRecursive(q1, q2, radius, node.left) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || this.queryVisibilityRecursive(q1, q2, radius, node.right));
            } else if (q1LeftOfI <= 0 && q2LeftOfI <= 0) {
                return this.queryVisibilityRecursive(q1, q2, radius, node.right) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || this.queryVisibilityRecursive(q1, q2, radius, node.left));
            } else if (q1LeftOfI >= 0 && q2LeftOfI <= 0) {
                /* One can see through obstacle from left to right. */
                return this.queryVisibilityRecursive(q1, q2, radius, node.left) && this.queryVisibilityRecursive(q1, q2, radius, node.right);
            } else {
                let point1LeftOfQ: number = Vector2.leftOf(q1, q2, obstacle1.point_);
                let point2LeftOfQ: number = Vector2.leftOf(q1, q2, obstacle2.point_);
                let invLengthQ: number = 1 / Vector2.absSq(q2.moins(q1));

                return (point1LeftOfQ * point2LeftOfQ >= 0 && RVOMath.sqr(point1LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && RVOMath.sqr(point2LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && this.queryVisibilityRecursive(q1, q2, radius, node.left) && this.queryVisibilityRecursive(q1, q2, radius, node.right));
            }
        }
    }
}