import {Vector2} from "./Vector2";

export class RoadmapVertex {
    position : Vector2;
    neighbors: number[];
    distToGoal: number[];
}