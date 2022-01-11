import {Simulator} from "./Simulator";
import {RoadmapVertex} from "./RoadmapVertex";
import {KeyValuePair} from "./KeyValuePair";
import {Vector2} from "./Vector2";
import {Multimap} from "./Multimap";

export class Dijkstra {
    public static buildRoadmap(sim: Simulator , roadmap:RoadmapVertex[]): void {
        for (let i = 0; i <roadmap.length; ++i) {
            for (let j = 0; j < roadmap.length; ++j) {
                if (sim.queryVisibility(roadmap[i].position, roadmap[j].position, sim.getAgentRadius(0))) {
                    roadmap[i].neighbors.push(j);
                }
            }

            /*
             * Initialize the distance to each of the four goal vertices at infinity
             * (9e9f).
             */
            for (let k = 0 ; k < 4 ; ++k) {
                roadmap[i].distToGoal[k] = Number.POSITIVE_INFINITY; 
            }
        }

        for (let i = 0; i < 4; ++i) {
            //std:: multimap < float, int > Q;
            let Q: Multimap = new Multimap();
            // std:: vector < std:: multimap < float, int >:: iterator > posInQ(roadmap.size(), Q.end());
            let posInQ: Multimap=new Multimap();

            roadmap[i].distToGoal[i] = 0;
            // posInQ[i] = Q.insert(std:: make_pair(0.0f, i));
          
            posInQ[i] = Q.keyValue.push(new KeyValuePair(0, i));

            while (Q.keyValue.length!=0) {
               // const u = Q.begin() - > second;
              //  Q.erase(Q.begin());
             //   posInQ[u] = Q.end();

                let u: number = Q.keyValue[0].Value;
                Q.keyValue.shift();
                posInQ[u] = Q.keyValue[Q.keyValue.length - 1]; 

                for (let j = 0; j < roadmap[u].neighbors.length; ++j) {
                    let v = roadmap[u].neighbors[j];
                    let dist_uv = Vector2.abs(roadmap[v].position.moins( roadmap[u].position));

                    if (roadmap[v].distToGoal[i] > roadmap[u].distToGoal[i] + dist_uv) {
                        roadmap[v].distToGoal[i] = roadmap[u].distToGoal[i] + dist_uv;

                        if (posInQ[v] == Q.keyValue[Q.keyValue.length - 1]) {
                            posInQ[v] = Q.keyValue.push(new KeyValuePair(roadmap[v].distToGoal[i], v));
                        }
                        else {
                          //  Q.erase(posInQ[v]);
                            Q.keyValue.push(new KeyValuePair(roadmap[v].distToGoal[i], v));
                        }
                    }
                }
            }
        }
    }
}