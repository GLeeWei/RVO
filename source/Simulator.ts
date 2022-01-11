import {Vector2} from "./Vector2";
import {Agent} from "./Agent";
import {KdTree} from "./KdTree";
import {Obstacle} from "./Obstacle";
import {Line} from "./Line";

export class Simulator {

    private static _instance: Simulator;
    private totalId: number = 0;
    private agentNoToIndexMap_: Map<number, number>;
    private indexToAgentNoMap_: Map<number, number>;

    defaultAgent_: Agent;
    agents_: Agent[];

    time_: number = 0;
    timeStep_: number;

    obstacles_: Obstacle[];
    kdTree_: KdTree;

    private constructor() {
        this.agents_ = [];
        this.agentNoToIndexMap_ = new Map<number, number>();
        this.indexToAgentNoMap_ = new Map<number, number>();
        this.obstacles_ = [];
        this.time_ = 0;
        this.defaultAgent_ = null;
        this.kdTree_ = new KdTree();
        this.timeStep_ = 1;
    }

    static get Instance(): Simulator {
        if (null == Simulator._instance)
            Simulator._instance = new Simulator();
        return Simulator._instance;
    }

    clear() {
        /*this['globalTime_'] = 0x0,
        this['timeStep_'] = 0.1,
        this['SetNumWorkers'](0xa);*/
        this.agentNoToIndexMap_.clear();
        this.indexToAgentNoMap_.clear();
        this.agents_.length = 0;
        this.obstacles_.length = 0;
        this.time_ = 0;
        this.defaultAgent_ = null;
        this.kdTree_ = new KdTree();
        this.timeStep_ = 1;
        this.totalId = 0;
    }

    doStep() {
        this.updateDeleteAgent();
        // console.log("ds la simu");
        this.kdTree_.buildAgentTree();

        for (let i = 0; i < this.getNumAgents(); ++i) {
            this.agents_[i].computeNeighbors();
            this.agents_[i].computeNewVelocity();
        }
        for (let i = 0; i < this.getNumAgents(); ++i) {
            this.agents_[i].update();
        }

        this.time_ += this.timeStep_;
    }

    processObstacles() {
        this.kdTree_.buildObstacleTree();
    }

    queryVisibility(point1: Vector2, point2: Vector2, radius: number) {
        return this.kdTree_.queryVisibility(point1, point2, radius);
    }

    addObstacle(vertices: Vector2[]): number {

        if (vertices.length < 2) {
            return -1;
        }

        let obstacleNo = this.obstacles_.length;

        for (let i = 0; i < vertices.length; ++i) {
            let obstacle: Obstacle = new Obstacle();
            obstacle.point_ = vertices[i];

            if (i != 0) {
                obstacle.prevObstacle_ = this.obstacles_[this.obstacles_.length - 1];
                obstacle.prevObstacle_.nextObstacle_ = obstacle;
            }

            if (i == vertices.length - 1) {
                obstacle.nextObstacle_ = this.obstacles_[obstacleNo];
                obstacle.nextObstacle_.prevObstacle_ = obstacle;
            }

            obstacle.unitDir_ = Vector2.normalize(vertices[(i == vertices.length - 1 ? 0 : i + 1)].moins(vertices[i]));

            if (vertices.length == 2) {
                obstacle.isConvex_ = true;
            } else {
                obstacle.isConvex_ = (Vector2.leftOf(vertices[(i == 0 ? vertices.length - 1 : i - 1)], vertices[i], vertices[(i == vertices.length - 1 ? 0 : i + 1)]) >= 0);
            }

            obstacle.id_ = this.obstacles_.length;

            this.obstacles_.push(obstacle);
        }

        return obstacleNo;
    }


    addAgent(position: Vector2 | Agent) {
        let agent: Agent;
        if (position instanceof Agent) {
            agent = position;
        } else {
            if (this.defaultAgent_ == null) {
                return -1;
            }
            agent = new Agent();
            agent.position_ = position;
            agent.maxNeighbors_ = this.defaultAgent_.maxNeighbors_;
            agent.maxSpeed_ = this.defaultAgent_.maxSpeed_;
            agent.neighborDist_ = this.defaultAgent_.neighborDist_;
            agent.radius_ = this.defaultAgent_.radius_;
            agent.timeHorizon_ = this.defaultAgent_.timeHorizon_;
            agent.timeHorizonObst_ = this.defaultAgent_.timeHorizonObst_;
            agent.velocity_ = this.defaultAgent_.velocity_;

        }
        agent.id_ = this.totalId++;
        this.agents_.push(agent);
        this.onAddAgent();
        return agent.id_;
    }

    onAddAgent() {
        let len = this.agents_.length;
        if (0 < len) {
            let idx = len - 1;
            let id_ = this.agents_[idx].id_;
            this.agentNoToIndexMap_.set(id_, idx);
            this.indexToAgentNoMap_.set(idx, id_);
        }
    }

    delAgent(agentNo: number) {
        let idx = this.agentNoToIndexMap_.get(agentNo);
        this.agents_[idx].needDelete_ = true;
    }

    onDelAgent() {
        this.agentNoToIndexMap_.clear();
        this.indexToAgentNoMap_.clear();
        for (let i = 0, len = this.agents_.length; i < len; i++) {
            let id_ = this.agents_[i].id_;
            this.agentNoToIndexMap_.set(id_, i);
            this.indexToAgentNoMap_.set(i, id_);
        }
    }

    setAgentDefaults(neighborDist: number, maxNeighbors: number, timeHorizon: number, timeHorizonObst: number,
                     radius: number, maxSpeed: number, velocity: Vector2 = new Vector2(0, 0)): void {
        if (this.defaultAgent_ == null)
            this.defaultAgent_ = new Agent();

        this.defaultAgent_.maxNeighbors_ = maxNeighbors;
        this.defaultAgent_.maxSpeed_ = maxSpeed;
        this.defaultAgent_.neighborDist_ = neighborDist;
        this.defaultAgent_.radius_ = radius;
        this.defaultAgent_.timeHorizon_ = timeHorizon;
        this.defaultAgent_.timeHorizonObst_ = timeHorizonObst;
        this.defaultAgent_.velocity_ = velocity;
    }

    setAgentPrefVelocity(agentNo: number, prefVelocity: Vector2) {
        let idx = this.agentNoToIndexMap_.get(agentNo);
        this.agents_[idx].prefVelocity_ = prefVelocity;
        // console.log(prefVelocity.x_, prefVelocity.y_);
    }

    setTimeStep(v: number): void {
        this.timeStep_ = v;
    }

    setAgentMaxSpeed(agentNo: number, maxSpeed: number) {
        let idx = this.agentNoToIndexMap_.get(agentNo);
        this.agents_[idx].maxSpeed_ = maxSpeed;
    }

    setAgentRadius(agentNo: number, radius: number) {
        let idx = this.agentNoToIndexMap_.get(agentNo);
        this.agents_[idx].radius_ = radius;
    }

    getAgentOrientation(agentNo: number): number {
        let idx = this.agentNoToIndexMap_.get(agentNo);
        return this.agents_[idx].orientation_;
    }

    getOrca(agentNo: number): Line[] {
        let idx = this.agentNoToIndexMap_.get(agentNo);
        return this.agents_[idx].orcaLines_;
    }

    getAgentPositionScreen(agentNo: number): Vector2 {
        let idx = this.agentNoToIndexMap_.get(agentNo);
        return this.agents_[idx].positionScreen_;
    }

    getAgentPosition(agentNo: number): Vector2 {
        let idx = this.agentNoToIndexMap_.get(agentNo);
        return this.agents_[idx].position_;
    }

    setAgentPosition(agentNo: number, pos: Vector2): void {
        let idx = this.agentNoToIndexMap_.get(agentNo);
        this.agents_[idx].position_ = pos;
    }

    getNumAgents(): number {
        return this.agents_.length;
    }

    getAgentRadius(agentNo: number): number {
        let idx = this.agentNoToIndexMap_.get(agentNo);
        return this.agents_[idx].radius_;
    }

    private updateDeleteAgent() {
        let bNeedUpdateAgent = false;
        for (let i = 0, len = this.agents_.length; i < len;) {
            let agent = this.agents_[i];
            if (agent.needDelete_) {
                this.agents_.splice(i, 1);
                len--;
                bNeedUpdateAgent = true;
                continue;
            }
            i++;
        }
        bNeedUpdateAgent && this.onDelAgent();
    }

    setAgentEnabled(agentNo: number, value: boolean) {
        let idx = this.agentNoToIndexMap_.get(agentNo);
        this.agents_[idx].enabled_ = value;
    }
}