//
//  detection.cpp
//  PicoCameraInfo
//
//  Created by Azhar Aulia Saputra on 2019/01/28.
//  Copyright © 2019 Azhar Aulia Saputra. All rights reserved.
//

#include "detection.h"
#include "OpenGL.hpp"


int calculateSimilarity(struct gng *net, int cl, int t2){
    int sim = 0;
    double *v = vectorSubtractionD(net->label_norm[cl], net->normTriangle[t2], 3);
    //    double *v = vectorSubtractionD(net->normTriangle[cl], net->normTriangle[t2], 3);
    double a = normD(v, 3);
    if(a < 5) sim = 1;
    
    free(v);
    //    printf("%.2f\n", a);
    return sim;
}
void checkSimilarityFromNeighbor(struct gng *net, int id){
    int i,j,k,l;
    int cur_Label = net->triangle[id][5];
    for(k = 0; k < DIM; k++){
        for(j = 0; j < net->triangle_n; j++){
            int neighbor = 0;
            if(j!=id){
                for(l = 0; l < DIM; l++){
                    if(net->triangle[id][k] == net->triangle[j][l]){
                        neighbor = 1; l =DIM;
                    }
                }
            }if(neighbor == 1 && net->triangle[j][5] < 0 && net->triangle[j][0] != net->triangle[j][2]){
                if(calculateSimilarity(net, cur_Label, j)){
                    double *v1 = vectorScaleD(double(net->label_ct[cur_Label]), net->label_norm[cur_Label], 3);
                    double *v = vectorScaleD(1,net->normTriangle[j],3);
                    double *v2 = vectorAddD(v1, v, 3);
                    
                    net->triangle[j][5] = cur_Label;
                    net->label_list[cur_Label][net->label_ct[cur_Label]] = j;
                    net->label_ct[cur_Label]++;
                    net->label_area[cur_Label] += net->triangle_area[j];
                    
                    net->label_norm[cur_Label][0] = v2[0]/(double)(net->label_ct[cur_Label]);
                    net->label_norm[cur_Label][1] = v2[1]/(double)(net->label_ct[cur_Label]);
                    net->label_norm[cur_Label][2] = v2[2]/(double)(net->label_ct[cur_Label]);
                    
                    checkSimilarityFromNeighbor(net,j);
                    
                    j = net->triangle_n;
                    
                    free(v);
                    free(v1);
                    free(v2);
                }
            }
        }
    }
}
void gng_plane_clustering(struct gng *net){
    int i, j, k, l;
    int cur_Label = 0;
    net->label_num = 0;
    for(i = 0; i < GNGN*10; i++)
        net->triangle[i][5] = -1;
    for(i = 0; i < CN; i++){
        net->label_ct[i] = 0;
        net->label_area[i] = 0;
        for(j = 0; j < CN; j++){
            net->label_list[i][j] = -1;
        }
        for(j = 0; j < DIM; j++){
            net->label_norm[i][j] = 0;
        }
    }
    for(i = 0; i < net->triangle_n; i++){
        if(net->triangle[i][5] == -1){
            int new_label = 0;
            if(net->triangle[i][0] != net->triangle[i][2]){
                if(net->triangle[i][5] < 0){
                    new_label = 1;
                }
            }else{
                new_label = 2;
                //                continue;
            }
            if(new_label != 0){
                net->triangle[i][5] = cur_Label;
                net->label_list[cur_Label][net->label_ct[cur_Label]] = i;
                net->label_ct[cur_Label]++;
                net->label_area[cur_Label] += net->triangle_area[i];
                for(k = 0; k < DIM; k++){
                    net->label_norm[cur_Label][k] = net->normTriangle[i][k];
                }
            }
            
            if(new_label != 2)
                checkSimilarityFromNeighbor(net,i);
            
            if(new_label != 0){
                cur_Label++;
                net->label_num = cur_Label;
            }
        }
    }
    int sum=0;
    for(i = 0; i < CN; i++){
        sum += net->label_ct[i];
    }
    //    printf("total involved planes = %d, %d\n", sum, net->triangle_n);
}
void checkSimilarityFromNeighborLabel(struct gng *net, int id){
    int h,i,j,k,l,m,n;
    int cur_Object = net->label_list[id][GNGN-1];
    for (h = 0; h < net->label_ct[id]; h++){
        i = net->label_list[id][h];
        for(j = 0; j < DIM; j++){
            int Node1 = net->triangle[i][j];
            for(k = 0; k < net->label_num; k++){
                int neighbor = 0;
                if(k!=id){
                    for (l = 0; l < net->label_ct[k]; l++){
                        m = net->label_list[k][l];
                        for(n = 0; n < DIM; n++){
                            int Node2 = net->triangle[m][j];
                            if(Node1 == Node2){
                                neighbor = 1; n =DIM;
                            }
                        }
                    }
                }
                
                if(neighbor == 1 && ( net->label_area[k] < 10000 || net->label_ct[k] <= 5)){
                    if(net->label_list[k][GNGN-1] == -1){
                        net->label_list[k][GNGN-1] = cur_Object;
                        net->sus_object_list[cur_Object][net->sus_object_ct[cur_Object]] = k;
                        net->sus_object_ct[cur_Object]++;
                        for(n = 0; n < DIM; n++){
                            double min = net->sus_object_size[cur_Object][n][0],
                            max = net->sus_object_size[cur_Object][n][1];
                            
                            for(l = 0; l < net->label_ct[k]; l++){
                                int t = net->label_list[k][l];
                                for(m = 0; m < 3; m++){
                                    if(max < net->node[net->triangle[t][m]][n]){
                                        max = net->node[net->triangle[t][m]][n];
                                    }
                                    if(min > net->node[net->triangle[t][m]][n]){
                                        min = net->node[net->triangle[t][m]][n];
                                    }
                                }
                            }
                            net->sus_object_size[cur_Object][n][0] = min;
                            net->sus_object_size[cur_Object][n][1] = max;
                        }
                        
                        checkSimilarityFromNeighborLabel(net, k);
                        k = net->label_num;
                    }
                }
                
            }
        }
    }
}
void gng_suspected_object(struct gng *net){
    int i, j, k, l, m, n;
    int cur_Object = 0;
    net->sus_object_num = 0;
    for(i = 0; i < CN; i++){
        net->label_list[i][GNGN-1] = -1;
        net->sus_object_ct[i] = 0;
        for(j = 0; j < GNGN; j++){
            net->sus_object_list[i][j] = -1;
        }
        for(j = 0; j < DIM; j++){
            for(k = 0; k < 2; k++)
                net->sus_object_size[i][j][k] = 0;
        }
    }
    for(i = 0; i < net->label_num; i++){
        int inn = 0;
        if(net->label_area[i] < 10000 || net->label_ct[i] <= 5){
            int new_label = 0;
            if(net->label_list[i][GNGN-1] < 0){
                new_label = 1;
                net->label_list[i][GNGN-1] = cur_Object;
                net->sus_object_list[cur_Object][net->sus_object_ct[cur_Object]] = i;
                net->sus_object_ct[cur_Object]++;
                for(k = 0; k < DIM; k++){
                    double min = 10000, max = -10000;
                    for(l = 0; l < net->label_ct[i]; l++){
                        int t = net->label_list[i][l];
                        for(m = 0; m < 3; m++){
                            if(max < net->node[net->triangle[t][m]][k]){
                                max = net->node[net->triangle[t][m]][k];
                            }
                            if(min > net->node[net->triangle[t][m]][k]){
                                min = net->node[net->triangle[t][m]][k];
                            }
                        }
                    }
                    net->sus_object_size[cur_Object][k][0] = min;
                    net->sus_object_size[cur_Object][k][1] = max;
                    if(min == 10000 || max == -10000){
                        printf("test");
                    }
                }
            }
            
            checkSimilarityFromNeighborLabel(net,i);
            
            if(new_label == 1){
                cur_Object ++;
                net->sus_object_num = cur_Object;
            }
        }
        else if(net->label_area[i] > 70000 && net->label_ct[i] >= 15){
            double *u = (double *) malloc(sizeof (double) * 3);
            double *v = (double *) malloc(sizeof (double) * 3);
            u[0] = 0; u[1] = 1; u[2] = 0;
            v[0] = net->label_norm[i][0];
            v[1] = net->label_norm[i][1];
            v[2] = net->label_norm[i][2];
            double a = acos((dotProductParD(u, v, 3) / (normD(u, 3) * normD(v, 3))));
            
            if(a < 10/rad){
                int new_label = 0;
                
                if(net->label_list[i][GNGN-1] < 0){
                    inn = 1;
                    new_label = 1;
                    net->label_list[i][GNGN-1] = cur_Object;
                    net->sus_object_list[cur_Object][net->sus_object_ct[cur_Object]] = i;
                    net->sus_object_ct[cur_Object]++;
                    for(k = 0; k < DIM; k++){
                        double min = 10000, max = -10000;
                        for(l = 0; l < net->label_ct[i]; l++){
                            int t = net->label_list[i][l];
                            for(m = 0; m < 3; m++){
                                if(max < net->node[net->triangle[t][m]][k]){
                                    max = net->node[net->triangle[t][m]][k];
                                }
                                if(min > net->node[net->triangle[t][m]][k]){
                                    min = net->node[net->triangle[t][m]][k];
                                }
                            }
                        }
                        net->sus_object_size[cur_Object][k][0] = min;
                        net->sus_object_size[cur_Object][k][1] = max;
                        
                    }
                    
                    net->sus_object_size[cur_Object][1][1] = net->sus_object_size[cur_Object][1][0] + 15;
                    
                    //                    printf("test _ %d, %d\n",cur_Object,i);
                    net->sus_object_list[cur_Object][GNGN-1] = 2;
                }
                if(new_label == 1){
                    cur_Object ++;
                    net->sus_object_num = cur_Object;
                }
            }
        }
        
        //        if(net->sus_object_list[cur_Object-1][GNGN-1] == 2 && inn != 1){
        //            printf("test\n");
        //        }
        
        //        for(int ii = 0; ii < net->sus_object_num; ii++)
        //
        //            printf("%d: %d\n",ii, net->sus_object_list[ii][GNGN-1]);
    }
    int ii, num;
    net->sus_node_num = 0;
    for(i = 0; i < net->sus_object_num; i++){
        
        //        printf("%d: %d\n",i, net->sus_object_list[i][GNGN-1]);
        if(net->sus_object_list[i][GNGN-1] != 2){
            net->sus_object_list[i][GNGN-1] = 0;
            double x_pos = (net->sus_object_size[i][0][0]+net->sus_object_size[i][0][1])/2;
            double x_size = abs (net->sus_object_size[i][0][0] - net->sus_object_size[i][0][1]);
            double z_pos = (net->sus_object_size[i][2][0]+net->sus_object_size[i][2][1])/2;
            double y_pos = (net->sus_object_size[i][1][0]+net->sus_object_size[i][1][1])/2;
            if(y_pos > GROUND_LIMIT && x_pos < 300 && x_pos > -300 && z_pos < 2500 && x_size < 800){//} && x_size > 200){
                net->sus_object_list[i][GNGN-1] = 1;
                
                for(ii=0; ii < net->sus_object_ct[ii]; ii++){
                    k = net->sus_object_list[i][ii];
                    for(l = 0; l < net->label_ct[k]; l++){
                        int t = net->label_list[k][l];
                        for(m = 0; m < 3; m++){
                            int n = net->triangle[t][m];
                            num = net->sus_node_num;
                            net->sus_node_num ++;
                            net->sus_node[num] = n;
                            for(int iii = 0; iii < num; iii++ ){
                                if(n == net->sus_node[iii]){
                                    net->sus_node_num--;
                                    continue;
                                }
                            }
                        }
                    }
                }
                
            }
        }
        else if(net->sus_object_list[i][GNGN-1] == 2){
            //            printf("test %d\n",i);
        }
    }
}


void searchConnectedRung(int id, struct gng *net, double *n, int n1, int n2, bool &connected){
    int i;
    //    printf("n = %.2f\t%.2f\t%.2f\tc = %d\n", n[0], n[1], n[2], cNodeRung);
    
    for(i = 0; i < net->node_n; i++){
        if(net->rung_nodeCon[id][i] == 0 && (net->edge[i][n1] == 1 || net->edge[n1][i] == 1)){
            double *r = vectorSubtractionD(net->node[n1], net->node[i], 3);
            double *v = vectorScaleD(dotProductParD(r, n, 3), n, 3);
            double *dR = vectorSubtractionD(r, v, 3);
            double d = normD(dR, 3);
            if(d < 15){
                // find next connection;
                
                net->rung_nodeCon[id][i] = 1;
                if(d >= 7) net->rung_nodeCon[id][i] = 2;
                
                net->rung_node_list[id][net->rung_ct[id]] = i;
                if(i == n2){
                    connected = true;
                }
                net->rung_ct[id] ++;
                
                searchConnectedRung(id, net, n, i, n2, connected);
            }
        }
    }
}
double **evaluateInlierOutlierRung(struct gng *net, int n1, int n2){
    int i,j,k,l;
    double* v1 = (double *) malloc(sizeof (double ) * 3);
    double* v2 = (double *) malloc(sizeof (double ) * 3);
    double **val = (double **) malloc(sizeof (double *) * 3);
    for(int i = 0; i < 3; i++)
        val[i] = (double *) malloc(sizeof (double ) * 3);
    double *n = vectorUnitD(vectorSubtractionD(net->node[n1], net->node[n2], 3));
    v1[0]=0; v1[1]=0; v1[2]=0;
    v2[0]=0; v2[1]=0; v2[2]=0;
    v2 = vectorSubtractionD(net->node[n1], net->node[n2], 3);
    int id = net->rung_num;
    for(i = 0; i < GNGN; i++){
        net->rung_nodeCon[id][i] = 0;
    }
    bool connected = false;
    
    net->rung_ct[id] = 0;
    net->rung_node_list[id][0] = n1;
    net->rung_nodeCon[id][n1] = 1;
    
    net->rung_ct[id] ++;
        searchConnectedRung(id, net, n, n1, n2, connected);
    //    printf("n = %.2f\t%.2f\t%.2f\tc = %d\n", n[0], n[1], n[2], cNodeRung);
    double cval = 0;
    double cvalr = 0;
    if(connected == true){
        for(i = 0; i < net->rung_ct[id]; i++){
            if(net->rung_nodeCon[id][net->rung_node_list[id][i]] == 1) cval++;
            else if(net->rung_nodeCon[id][net->rung_node_list[id][i]] == 2) cvalr++;
        }
        
        if(cval > 12){
            
            double *r = vectorSubtractionD(net->node[n1], net->node[net->rung_node_list[id][i]], 3);
            double *v = vectorScaleD(dotProductParD(r, n, 3), n, 3);
            //            double *b = vectorSubtractionD(net->node[n1], v1, 3);
            for(i = 0; i < net->rung_ct[id]; i++){
                if(net->rung_nodeCon[id][net->rung_node_list[id][i]] == 1){
                    double *r = vectorSubtractionD(net->node[n1], net->node[net->rung_node_list[id][i]], 3);
                    double *v = vectorScaleD(dotProductParD(r, n, 3), n, 3);
                    double *dR = vectorSubtractionD(r, v, 3);
                    double d = normD(dR, 3);
                    
                    if(normD(vectorSubtractionD(v, v2, 3), 3) >= normD(vectorSubtractionD(v, v1, 3), 3)){
                        if(normD(vectorSubtractionD(v, v2, 3), 3) > normD(vectorSubtractionD(v1, v2, 3), 3))
                            v1 = vectorScaleD(1, v, 3);
                    }else if(normD(vectorSubtractionD(v, v2, 3), 3) <= normD(vectorSubtractionD(v, v1, 3), 3)){
                        if(normD(vectorSubtractionD(v, v1, 3), 3) > normD(vectorSubtractionD(v1, v2, 3), 3))
                            v2 = vectorScaleD(1, v, 3);
                    }
                    
                    free(r);
                    free(v);
                    free(dR);
                }
            }
        }
    }
    
    val[1] = vectorSubtractionD(net->node[n1], v1, 3);
    val[2] = vectorSubtractionD(net->node[n1], v2, 3);
    double *v3 = vectorSubtractionD(val[1], val[2], 3);
    double L = normD(v3, 3)/100;
    if(cval > 9 && cval/cvalr > 3){
        //        printf("%.2f\t%.2f\t%.2f\n", v1[0], v1[1], v1[2]);
        //        printf("%.2f\t%.2f\t%.2f\n", v2[0], v2[1], v2[2]);
        //
        //
        //                printf("t = %.2f\t%.2f\t%.2f\n", val[1][0], val[1][1], val[1][2]);
        //                printf("t = %.2f\t%.2f\t%.2f\n", val[2][0], val[2][1], val[2][2]);
        if(L > 0.3){
            val[0][0] = double((cval/(1+cvalr)))/L;
            //            val[0][0] = double((cval/(1+cvalr))*(cval/(1+cvalr)))/L;
            val[0][2] = cvalr;
            
            val[0][1] = cval;
        }else{
            val[0][0] = 0;
            val[0][2] = 0;
            val[0][1] = cval;
        }
        
        //        printf("t = %.2f\t%.2f\t%.2f\n", val[1][0], val[1][1], val[1][2]);
        //        printf("t = %.2f\t%.2f\t%.2f\n", val[2][0], val[2][1], val[2][2]);
        //        printf("t = %.2f\t%.2f\t%.2f\n", val[0][0], val[0][1], val[0][2]);
    }else{
        val[0][0] = 0;
        val[0][2] = 0;
        val[0][1] = cval;
    }
    
    //    val = cval;
    
    free(n);
    free(v1);
    free(v2);
    free(v3);
    return val;
}
void updateConnectedRung(int id, struct gng *net, double *n, int n1){
    int i;
    //    printf("n = %.2f\t%.2f\t%.2f\tc = %d\n", n[0], n[1], n[2], cNodeRung);
    
    for(i = 0; i < net->node_n; i++){
        if(net->rung_nodeCon[id][i] == 0 && (net->edge[i][n1] == 1 || net->edge[n1][i] == 1)){
            double *r = vectorSubtractionD(net->rung_node[id][0], net->node[i], 3);
            double *v = vectorScaleD(dotProductParD(r, n, 3), n, 3);
            double *dR = vectorSubtractionD(r, v, 3);
            double d = normD(dR, 3);
            if(d < 15){
                // find next connection;
                
                net->rung_nodeCon[id][i] = 1;
                if(d >= 7) net->rung_nodeCon[id][i] = 2;
                
                net->rung_node_list[id][net->rung_ct[id]] = i;
                net->rung_ct[id] ++;
                //                printf("%d, ",net->rung_ct[id]);
                updateConnectedRung(id, net, n, i);
            }
        }
    }
}
void delete_rung(struct gng *net, int i){
    int j,k,l;
    for(j=i;j<net->rung_num;j++){
        for(k = 0; k < 3; k++){
            net->rung_eval[j][k] = net->rung_eval[j+1][k] ;
            net->rung_node[j][0][k] = net->rung_node[j+1][0][k];
            net->rung_node[j][1][k] = net->rung_node[j+1][1][k];
        }
        net->rung_ct[j] = net->rung_ct[j+1];
        net->rung_age[j] = net->rung_age[j+1];
        for (k = 0; k < GNGN; k++) {
            net->rung_nodeCon[j][k] = net->rung_nodeCon[j+1][k];
            net->rung_node_list[j][k] = net->rung_node_list[j+1][k];
        }
        
        net->rung_affordance_ct[j] = net->rung_affordance_ct[j+1];
        for(k = 0; k < RN; k++){
            for(l = 0; l < 3; l++){
                net->rung_affordance_point[j][k][l] = net->rung_affordance_point[j+1][k][l];
            }
        }
    }
    net->rung_num--;
    net->rung_ct[net->rung_num] = 0;
    net->rung_affordance_ct[net->rung_num] = 0;
}
int calc_similirity(struct gng *net, int id){
    int i,j,k;
    int sim = 0;
    
    double* n = (double *) malloc(sizeof (double ) * 3);
    double* u = (double *) malloc(sizeof (double ) * 3);
    double* v = (double *) malloc(sizeof (double ) * 3);
    double* r = (double *) malloc(sizeof (double ) * 3);
    double* dR = (double *) malloc(sizeof (double ) * 3);
    double* V = (double *) malloc(sizeof (double ) * 3);
    for(i = 0; i < net->rung_num; i++){
        //        u = vectorSubtractionD(eval[1], eval[2], 3);
        if(id != i){
            u = vectorSubtractionD(net->rung_node[id][0], net->rung_node[id][1], 3);
            v = vectorSubtractionD(net->rung_node[i][0], net->rung_node[i][1], 3);
            double a = dotProductParD(u, v, 3);
            double b = normD(u, 3) * normD(v, 3);
            double simVal = a / b;
            
            if(simVal > 0.9 || simVal < -0.9){
                
                //            printf("k = %.2f\t%.2f\t%.2f\n", u[0], u[1], u[2]);
                //            printf("k = %.2f\t%.2f\t%.2f\n", v[0], v[1], v[2]);
                n = vectorUnitD(v);
                u = vectorAddD(net->rung_node[id][0], net->rung_node[id][1], 3);
                v = vectorScaleD(0.5, u, 3);
                r = vectorSubtractionD(net->rung_node[i][0], v, 3);
                V = vectorScaleD(dotProductParD(r, n, 3), n, 3);
                dR = vectorSubtractionD(r, V, 3);
                double d = normD(dR, 3);
                
                if(d < 16){
                    sim = 1;
                    //                printf("e = %.2f\t%.2f\t%.2f\n", eval[0][0], eval[0][1], eval[0][2]);
                    //                printf("e = %.2f\t%.2f\t%.2f\n", eval[1][0], eval[1][1], eval[1][2]);
                    //                printf("e = %.2f\t%.2f\t%.2f\n", eval[2][0], eval[2][1], eval[2][2]);
                    if((net->rung_eval[id][1] >= net->rung_eval[i][1]-2) && (net->rung_eval[id][0] > net->rung_eval[i][0])){
                        if(net->rung_age[id] < net->rung_age[i])
                            net->rung_age[id] = net->rung_age[i];
                        delete_rung(net, i);
                    }
                    i = net->rung_num;
                }
                
            }
        }
    }
    free(u);
    free(n);
    free(v);
    free(V);
    free(dR);
    free(r);
    return sim;
}

void rung_update_position(struct gng *net){
    int i,j,k,m;
    int n1, n2;
    double* v1 = (double *) malloc(sizeof (double ) * 3);
    double* v2 = (double *) malloc(sizeof (double ) * 3);
    double *val = (double *) malloc(sizeof (double ) * 3);
    
    for(i = 0; i < net->rung_num; i++){
        int cval = 0, cvalr = 0;
        
        v1[0]=0; v1[1]=0; v1[2]=0;
        v2 = vectorSubtractionD(net->rung_node[i][0], net->rung_node[i][1], 3);
        
        double l = normD(v2, 3);
        double *n = vectorUnitD(v2);
        double *errPoint_rung1 = (double *) malloc(sizeof (double ) * 3);
        double *errPoint_rung2 = (double *) malloc(sizeof (double ) * 3);
        errPoint_rung1[0] = 0; errPoint_rung1[1] = 0; errPoint_rung1[2] = 0;
        errPoint_rung2[0] = 0; errPoint_rung2[1] = 0; errPoint_rung2[2] = 0;
        
        
        int rung_ct = net->rung_ct[i];
        int rung_node_list[100];
        
        //        for(j = 0; j < net->rung_ct[i]; j++){
        //            printf("%d, ", net->rung_node_list[i][j]);
        //        }
        //
        //        printf("\n");
        //        printf("\n");
        //        printf("%d->%d\n", i, net->rung_ct[i]);
        for(j = 0; j < net->rung_ct[i]; j++){
            rung_node_list[j] = net->rung_node_list[i][j];
        }for(j = 0; j < GNGN; j++){
            net->rung_nodeCon[i][j] = 0;
        }
        net->rung_ct[i] = 0;
        for(j = 0; j < rung_ct; j++){
            if(net->rung_nodeCon[i][rung_node_list[j]] == 0){
                double *r = vectorSubtractionD(net->rung_node[i][0], net->node[rung_node_list[j]], 3);
                double *v = vectorScaleD(dotProductParD(r, n, 3), n, 3);
                double *dR = vectorSubtractionD(r, v, 3);
                double d = normD(dR, 3);
                if(d < 15){
                    // find next connection;
                    
                    net->rung_nodeCon[i][rung_node_list[j]] = 1;
                    if(d >= 7) net->rung_nodeCon[i][rung_node_list[j]] = 2;
                    
                    net->rung_node_list[i][net->rung_ct[i]] = rung_node_list[j];
                    net->rung_ct[i] ++;
                    
                    //                    printf("\n basic = %d, ",net->rung_ct[i]);
                    updateConnectedRung(i, net, n, rung_node_list[j]);
                    
                    //                    printf("\n real = %d, ",net->rung_ct[i]);
                }else{
                    
                }
            }
        }
        //        printf("%d->%d\n", i, net->rung_ct[i]);
//        if(net->rung_ct[i] == 0){
//            for(j = 0; j < net->node_n; j++){
//                //            if(rangeArea(net->node[j][0],net->node[j][1],net->node[j][2],0,30,170,250,250,50) == 1){
//                int in = 0;
//                for(m = 0; m < net->sus_object_num; m++){
//                    if(net->sus_object_list[m][GNGN-1] == 2){
//                        if(rangeArea2(net->node[j][0],net->node[j][1],net->node[j][2], net->sus_object_size[m][0][0]-20, net->sus_object_size[m][0][1]+20,
//                                      net->sus_object_size[m][1][0]-20, net->sus_object_size[m][1][1]+20,
//                                      net->sus_object_size[m][2][0]-20, net->sus_object_size[m][2][1]+20) == 1){
//                            m = net->sus_object_num;
//                            in = 0;
//                        }
//                    }else if(net->sus_object_list[m][GNGN-1] == 1 && net->node[j][2] < 800){
//                        if(rangeArea2(net->node[j][0],net->node[j][1],net->node[j][2], net->sus_object_size[m][0][0]-20, net->sus_object_size[m][0][1]+20,
//                                      net->sus_object_size[m][1][0]-20, net->sus_object_size[m][1][1]+20,
//                                      net->sus_object_size[m][2][0]-20, net->sus_object_size[m][2][1]+20) == 1){
//                            m = net->sus_object_num;
//                            in = 1;
//                        }
//                    }
//                }
//
//                if(in == 1){
//                    double *r = vectorSubtractionD(net->rung_node[i][0], net->node[j], 3);
//                    double *rr = vectorSubtractionD(net->rung_node[i][1], net->node[j], 3);
//                    double *v = vectorScaleD(dotProductParD(r, n, 3), n, 3);
//                    double *dR = vectorSubtractionD(r, v, 3);
//                    double d = normD(dR, 3);
//                    if(d < 15 && (l >= normD(r,3)) && (l >= normD(rr,3))){
//                        net->rung_nodeCon[i][j] = 1;
//                        if(d >= 7) net->rung_nodeCon[i][j] = 2;
//
//                        net->rung_node_list[i][net->rung_ct[i]] = j;
//                        net->rung_ct[i] ++;
//                    }
//
//                    free(rr);
//                    free(r);
//                    free(v);
//                    free(dR);
//                }
//            }
//        }
        //
        //        for(j = 0; j < net->rung_ct[i]; j++){
        //            printf("%d, ", net->rung_node_list[i][j]);
        //        }
        //
        //        printf("\n");
        
        
        for(j = 0; j < net->rung_ct[i]; j++){
            if(net->rung_nodeCon[i][net->rung_node_list[i][j]] == 1) cval++;
            else if(net->rung_nodeCon[i][net->rung_node_list[i][j]] == 2) cvalr++;
        }
        j = 0;
        double *r = vectorSubtractionD(net->rung_node[i][0], net->node[net->rung_node_list[i][j]], 3);
        double *v = vectorScaleD(dotProductParD(r, n, 3), n, 3);
        
        v1 = vectorScaleD(1, v, 3);
        v2 = vectorScaleD(1, v, 3);
        
        for(j = 1; j < net->rung_ct[i]; j++){
            r = vectorSubtractionD(net->rung_node[i][0], net->node[net->rung_node_list[i][j]], 3);
            v = vectorScaleD(dotProductParD(r, n, 3), n, 3);
            if(normD(vectorSubtractionD(v, v2, 3), 3) >= normD(vectorSubtractionD(v, v1, 3), 3)){
                if(normD(vectorSubtractionD(v, v2, 3), 3) > normD(vectorSubtractionD(v1, v2, 3), 3))
                    v1 = vectorScaleD(1, v, 3);
            }else if(normD(vectorSubtractionD(v, v2, 3), 3) <= normD(vectorSubtractionD(v, v1, 3), 3)){
                if(normD(vectorSubtractionD(v, v1, 3), 3) > normD(vectorSubtractionD(v1, v2, 3), 3))
                    v2 = vectorScaleD(1, v, 3);
            }
        }
        
        double *b1 = vectorSubtractionD(net->rung_node[i][0], v1, 3);
        double *b2 = vectorSubtractionD(net->rung_node[i][0], v2, 3);
        
        //        printf("\nR0 = %.2f, %.2f, %.2f : %.2f, %.2f, %.2f\n",net->rung_node[i][0][0],net->rung_node[i][0][1],net->rung_node[i][0][2],
        //               net->rung_node[i][1][0],net->rung_node[i][1][1],net->rung_node[i][1][2]);
        //        printf("R1 = %.2f, %.2f, %.2f : %.2f, %.2f, %.2f\n",b1[0],b1[1],b1[2],b2[0],b2[1],b2[2]);
        //        v1[0]=0; v1[1]=0; v1[2]=0;
        //        v2[0]=0; v2[1]=0; v2[2]=0;
        
        for(j = 0; j < net->rung_ct[i]; j++){
            r = vectorSubtractionD(b1, net->node[net->rung_node_list[i][j]], 3);
            v = vectorScaleD(dotProductParD(r, n, 3), n, 3);
            double *dR = vectorSubtractionD(r, v, 3);
            double d = normD(dR, 3);
                
            //            if(net->rung_nodeCon[i][net->rung_node_list[i][j]] == 2){
            double *r2 = vectorSubtractionD(b2, net->node[net->rung_node_list[i][j]], 3);
            double *vv2 = vectorScaleD(dotProductParD(r2, n, 3), n, 3);
            double *err1 = vectorScaleD(normD(vv2, 3)/l, dR, 3);
            double *err2 = vectorScaleD(normD(v, 3)/l, dR, 3);
            errPoint_rung1 = vectorAddD(errPoint_rung1, err1, 3);
            errPoint_rung2 = vectorAddD(errPoint_rung2, err2, 3);
            free(r2);
            free(vv2);
            free(err1);
            free(err2);
            free(dR);
        }
        free(r);
        free(v);
        
        
        double *v3 = vectorSubtractionD(b1, b2, 3);
        //        if(cval > 0){
        double L = normD(v3, 3)/100;
        //        val[0] = double((cval/(1+cvalr))*(cval/(1+cvalr)))/L;
        val[0] = double((cval/(1+cvalr)))/L;
        val[2] = cvalr;
        val[1] = cval;
        
        errPoint_rung1 =vectorScaleD((1/(double)cval), errPoint_rung1, 3);
        errPoint_rung2 =vectorScaleD((1/(double)cval), errPoint_rung2, 3);
        //        }
        //        printf("update rung: %d, cval = %d, cvalr = %d, v1 = %.2f, %.2f, %.2f, v2 = %.2f, %.2f, %.2f\n", i, cval, cvalr, v1[0], v1[1], v1[2], v2[0], v2[1], v2[2]);
        //        printf("errPoint_rung1 = %.2f, %.2f, %.2f\n", errPoint_rung1[0], errPoint_rung1[1], errPoint_rung1[2]);
        //        printf("errPoint_rung2 = %.2f, %.2f, %.2f\n", errPoint_rung2[0], errPoint_rung2[1], errPoint_rung2[2]);
        
        for(j = 0; j < 3; j++){
            //            net->rung_node[i][0][j] = net->rung_node[i][0][j]-v1[j] - 0.99* errPoint_rung1[j];
            //            net->rung_node[i][1][j] = net->rung_node[i][0][j]-v2[j] - 0.99 * errPoint_rung2[j];
            net->rung_node[i][0][j] = b1[j] - 0.49* errPoint_rung1[j];
            net->rung_node[i][1][j] = b2[j] - 0.49 * errPoint_rung2[j];
            
            net->rung_eval[i][j] = val[j];
        }
        
        free(n);
        if(net->rung_ct[i] <= 0.001)
            delete_rung(net, i);
        else
            calc_similirity(net, i);
    }
    
    free(v1);
    free(v2);
}

double draw_RUNG(struct gng *net, int n1, int color){
    double val;
    int cval = 0;
    int i;
    
    glLineWidth(10);
    if(color == 0)
        glColor4f(1, 0, 1, 0.4);
    else if(color == 1)
        glColor4f(0, 0, 1, 0.4);
    glBegin(GL_LINES);
    glVertex3s(net->rung_node[n1][0][0], net->rung_node[n1][0][1], net->rung_node[n1][0][2]);
    glVertex3s(net->rung_node[n1][1][0], net->rung_node[n1][1][1], net->rung_node[n1][1][2]);
    glEnd();
    
    glLineWidth(1);
    
    val = cval;
    return val;
}
int rung_calc_similirity(struct gng *net, double **eval){
    int i,j,k;
    int sim = 0;
    
    double* n = (double *) malloc(sizeof (double ) * 3);
    double* u = (double *) malloc(sizeof (double ) * 3);
    double* v = (double *) malloc(sizeof (double ) * 3);
    double* r = (double *) malloc(sizeof (double ) * 3);
    double* dR = (double *) malloc(sizeof (double ) * 3);
    double* V = (double *) malloc(sizeof (double ) * 3);
    for(i = 0; i < net->rung_num; i++){
        u = vectorSubtractionD(eval[1], eval[2], 3);
        v = vectorSubtractionD(net->rung_node[i][0], net->rung_node[i][1], 3);
        double a = dotProductParD(u, v, 3);
        double b = normD(u, 3) * normD(v, 3);
        double simVal = a / b;
        if(simVal > 0.9 || simVal < -0.9){
            //            printf("k = %.2f\t%.2f\t%.2f\n", u[0], u[1], u[2]);
            //            printf("k = %.2f\t%.2f\t%.2f\n", v[0], v[1], v[2]);
            n = vectorUnitD(v);
            u = vectorAddD(eval[1], eval[2], 3);
            v = vectorScaleD(0.5, u, 3);
            r = vectorSubtractionD(net->rung_node[i][0], v, 3);
            V = vectorScaleD(dotProductParD(r, n, 3), n, 3);
            dR = vectorSubtractionD(r, V, 3);
            double d = normD(dR, 3);
            
            if(d < 16){
                sim = 1;
                //                printf("e = %.2f\t%.2f\t%.2f\n", eval[0][0], eval[0][1], eval[0][2]);
                //                printf("e = %.2f\t%.2f\t%.2f\n", eval[1][0], eval[1][1], eval[1][2]);
                //                printf("e = %.2f\t%.2f\t%.2f\n", eval[2][0], eval[2][1], eval[2][2]);
                //                if(eval[0][0] > net->rung_eval[i][0]){
                //                    for(j = 0; j < 3; j++){
                //                        net->rung_eval[i][j] = eval[0][j];
                //                        net->rung_node[i][0][j] = eval[1][j];
                //                        net->rung_node[i][1][j] = eval[2][j];
                //                    }
                //                    net->rung_ct[i] = net->rung_ct[net->rung_num];
                //                    if(net->rung_age[i] < INIT_RUNG_AGE)
                //                        net->rung_age[i] = INIT_RUNG_AGE;
                //
                //                    for (j = 0; j < GNGN; j++) {
                //                        net->rung_nodeCon[i][j] = net->rung_nodeCon[net->rung_num][j];
                //                        net->rung_node_list[i][j] = net->rung_node_list[net->rung_num][j];
                //                    }
                //                }
                i = net->rung_num;
            }
            
        }
    }
    free(u);
    free(n);
    free(v);
    free(V);
    free(dR);
    free(r);
    return sim;
}
int n_ind = 50;
void rung_calc_age(struct gng *net){
    int i,j,k;
    for(i = 0; i < net->rung_num; i++){
        if(net->rung_eval[i][0] <= 0.3  || (net->rung_eval[i][1]-net->rung_eval[i][2] < 7)){
           net->rung_age[i] --;
           if(net->rung_eval[i][0] <= 0.3 && (net->rung_eval[i][1]-net->rung_eval[i][2] < 7))
               net->rung_age[i] --;
        }
        else if(net->rung_eval[i][0] > 0.3) net->rung_age[i] ++;
        
        if(net->rung_age[i] > MAX_RUNG_AGE)
            net->rung_age[i] = MAX_RUNG_AGE;
        else if(net->rung_age[i] < 0)
            delete_rung(net, i);
    }
}
void generateMultiRung(struct gng *net){
    int i,j,k;
    int n1=0, n2=0;
    int mN1, mN2;
    int rank[RN];
    int node[n_ind][2];
    
    
    rung_update_position(net);
    rung_calc_age(net);
    double **eval = (double **) malloc(sizeof (double *) * n_ind);
    
    //    double **v;
    for(i = 0; i < n_ind/(net->rung_num+1); i++){
        int in = 0;
        int count = 0;
        while(net->obs_count > 0 && in != 1 && count <= net->node_n){
            n1 = rnd() * net->node_n;
            
            while(net->node[n1][1] < GROUND_LIMIT || net->node[n1][0] < -300 || net->node[n1][0] > 300 || in == 2){
                n1 = rnd() * net->node_n;
                in = 0;
            }
            in = 2;
            for(j = 0; j < net->sus_object_num; j++){
                
                if(net->sus_object_list[j][GNGN-1] == 2){
                    if(rangeArea2(net->node[n1][0],net->node[n1][1],net->node[n1][2], net->sus_object_size[j][0][0]-20, net->sus_object_size[j][0][1]+20,
                                  net->sus_object_size[j][1][0]-20, net->sus_object_size[j][1][1]+20,
                                  net->sus_object_size[j][2][0]-20, net->sus_object_size[j][2][1]+20) == 1){
                        j = net->sus_object_num;
                        in = 0;
                        continue;
                    }
                }
                if(net->sus_object_list[j][GNGN-1] == 1 && net->node[n1][2] < 600){
                    if(rangeArea2(net->node[n1][0],net->node[n1][1],net->node[n1][2], net->sus_object_size[j][0][0]-20, net->sus_object_size[j][0][1]+20,
                                  net->sus_object_size[j][1][0]-20, net->sus_object_size[j][1][1]+20,
                                  net->sus_object_size[j][2][0]-20, net->sus_object_size[j][2][1]+20) == 1){
                        j = net->sus_object_num;
                        in = 1;
                    }
                }
            }
            count++;
        }
        in = 0;
        count = 0;
        while(net->obs_count > 0 && in != 1 && count <= net->node_n){
                n2 = rnd() * net->node_n;
                while(net->node[n2][1] < GROUND_LIMIT || net->node[n1][0] < LEFT_LIMIT || net->node[n1][0] > RIGHT_LIMIT  || net->node[n1][0] > LENGTH_LIMIT || in == 2 || n1 == n2
                      || ((net->node[n2][0] < net->node[n1][0] - 20 || net->node[n2][0] > net->node[n1][0] + 20)
                          && (net->node[n2][1] < net->node[n1][1] - 20 || net->node[n2][1] > net->node[n1][1] + 20))){
                    n2 = rnd() * net->node_n;
                    in = 0;
                }
                in = 2;
                for(j = 0; j < net->sus_object_num; j++){
                    if(net->sus_object_list[j][GNGN-1] == 2){
                        if(rangeArea2(net->node[n2][0],net->node[n2][1],net->node[n2][2], net->sus_object_size[j][0][0]-20, net->sus_object_size[j][0][1]+20,
                                      net->sus_object_size[j][1][0]-20, net->sus_object_size[j][1][1]+20,
                                      net->sus_object_size[j][2][0]-20, net->sus_object_size[j][2][1]+20) == 1){
                            j = net->sus_object_num;
                            in = 0;
                            continue;
                        }
                    }
                    if(net->sus_object_list[j][GNGN-1] == 1 && net->node[n2][2] < 600){
                        if(rangeArea2(net->node[n2][0],net->node[n2][1],net->node[n2][2], net->sus_object_size[j][0][0]-20, net->sus_object_size[j][0][1]+20,
                                      net->sus_object_size[j][1][0]-20, net->sus_object_size[j][1][1]+20,
                                      net->sus_object_size[j][2][0]-20, net->sus_object_size[j][2][1]+20) == 1){
                            j = net->sus_object_num;
                            in = 1;
                        }
                    }
                }
            count++;
        }
        
        //        eval = evaluateInlierOutlierNode(net, n1, n2);
        eval = evaluateInlierOutlierRung(net, n1, n2);
        int sim = rung_calc_similirity(net, eval);
        int c = net->rung_num;
        if(sim == 0 && c < RN && eval[0][0] > 1){
            //            printf("k = %.2f\t%.2f\t%.2f\n", eval[1][0], eval[1][1], eval[1][2]);
            //            printf("k = %.2f\t%.2f\t%.2f\n", eval[0][0], eval[0][1], eval[0][2]);
            for(j = 0; j < 3; j++){
                net->rung_eval[c][j] = eval[0][j];
                net->rung_node[c][0][j] = eval[1][j];
                net->rung_node[c][1][j] = eval[2][j];
            }
            net->rung_age[c] = INIT_RUNG_AGE;
            net->rung_num++;
            
            //            printf("add rung: %d, cval = %.0f, cvalr = %.0f, v1 = %.2f, %.2f, %.2f, v2 = %.2f, %.2f, %.2f\n", c, eval[0][1], eval[0][2], eval[1][0], eval[1][1], eval[1][2], eval[2][0], eval[2][1], eval[2][2]);
        }
        //        printf("%.2f\n",eval[i]);
    }
    for(i = 0; i < net->rung_num; i++){
        int flag = 0;
        for(j = 0; j < i; j++){
            if(net->rung_eval[i][0] > net->rung_eval[rank[j]][0]){
                for(k = i-1; k >= j; k--){
                    rank[k+1] = rank[k];
                }
                rank[j] = i;
                j = i+1;
                flag = 1;
            }
        }
        if(flag == 0){
            rank[i] = i;
        }
    }
    //    for(i = 0; i < 10; i++){
    ////        printf("%.2f\n",eval[rank[i]]);
    //        if(net->rung_num <= i)
    //            continue;
    //        if(net->rung_eval[rank[i]][0] > 1)
    //            draw_RUNG(net, rank[i], 0);
    //        else
    //            draw_RUNG(net, rank[i], 1);
    //    }
    rung_affordance_detection(net);
    for(i = 0; i < net->rung_num; i++){
        if(net->rung_age[i] > THRES_RUNG_AGE)
            draw_RUNG(net, i, 0);
        //        else
        //            draw_RUNG(net, rank[i], 1);
    }
    
    char cbuff1[50];
    char cbuff2[10];
    char cbuff3[10];
    for(i = 0; i < net->rung_num; i++){
        sprintf(cbuff1, "(%d): %.2f, %.f, %.f, %d",i, net->rung_eval[i][0],net->rung_eval[i][1],net->rung_eval[i][2], net->rung_age[i]);
        //        sprintf(cbuff2, "%.f",net->rung_eval[i][1]);
        //        sprintf(cbuff3, "%.f",net->rung_eval[i][2]);
        float x = (net->rung_node[i][0][0] + net->rung_node[i][1][0])/2;
        float y = (net->rung_node[i][0][1] + net->rung_node[i][1][1])/2;
        float z = (net->rung_node[i][0][2] + net->rung_node[i][1][2])/2;
        
        glColor3f(0, 0, 0);
        //        drawBitmapText(cbuff1,x, y+6, z);
        drawBitmapText(cbuff1,x-10, y+10, z);
        //        drawBitmapText(cbuff3,x, y-6, z);
    }
    
    //    draw_RUNG(net, 0);
    free2d_double(eval);
}

void draw_rung_affordance(struct gng *net, int id){
    int j;
    glPointSize(10);
    glBegin(GL_POINTS);
    for(j = 0; j < net->rung_affordance_ct[id]; j++){
        if(net->rung_affordance_posi[id][j] == 1)
            glColor3f(0, 1,0);
        else
            glColor3f(1, 0,0);
        glVertex3f(net->rung_affordance_point[id][j][0], net->rung_affordance_point[id][j][1], net->rung_affordance_point[id][j][2]);
    }
    glEnd();
}
double range = 5;
void rung_affordance_detection(struct gng *net){
    int i,j,k,l;
    for(i = 0; i < net->rung_num; i++){
        if(net->rung_age[i] > MAX_RUNG_AGE-20){
            double *v = vectorSubtractionD(net->rung_node[i][0], net->rung_node[i][1], 3);
            double *n = vectorUnitD(v);
            double max = normD(v, 3);
            net->rung_affordance_ct[i] = 0;
        
            double *p = vectorScaleD((net->rung_affordance_ct[i])*(range), n, 3);
            double cMax = normD(p, 3);
            for(j = 0; j < 3; j++){
                net->rung_affordance_point[i][net->rung_affordance_ct[i]][j] = net->rung_node[i][0][j] - p[j];
            }
            net->rung_affordance_ct[i] ++;
            while(max > cMax){
                
                p = vectorScaleD((net->rung_affordance_ct[i])*(range), n, 3);
                cMax = normD(p, 3);
                for(j = 0; j < 3; j++){
                    net->rung_affordance_point[i][net->rung_affordance_ct[i]][j] = net->rung_node[i][0][j] - p[j];
                }
                
                
                net->rung_affordance_posi[i][net->rung_affordance_ct[i]] = 1;
                for(j = 0; j < net->rung_ct[i]; j++){
                    if(net->rung_nodeCon[i][j] == 2){
                        double *r = vectorSubtractionD(net->rung_affordance_point[i][net->rung_affordance_ct[i]], net->node[net->rung_node_list[i][j]], 3);
                        double *v1 = vectorScaleD(dotProductParD(r, n, 3), n, 3);
                        double d = normD(v1, 3);
                        if(d < 20){
                            net->rung_affordance_posi[i][net->rung_affordance_ct[i]] = 0;
                            j = GNGN;
                        }
                    }
                }
                
                for(k = 0; k < net->rung_num; k++){
                    if(i != k){
                        for(j = 0; j < net->rung_affordance_ct[k]; j++){
                            double *r = vectorSubtractionD(net->rung_affordance_point[i][net->rung_affordance_ct[i]], net->rung_affordance_point[k][j], 3);
//                            double *v1 = vectorScaleD(dotProductParD(r, n, 3), n, 3);
                            double d = normD(r, 3);
                            if(d < 20){
                                net->rung_affordance_posi[i][net->rung_affordance_ct[i]] = 0;
                                j = GNGN;
                                k = net->rung_num;
                            }
                        }
                    }
                }
                
                net->rung_affordance_ct[i] ++;
            }
            draw_rung_affordance(net, i);
            free(p);
            free(v);
            free(n);
        }
    }
}

