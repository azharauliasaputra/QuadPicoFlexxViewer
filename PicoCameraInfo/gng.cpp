/*
 *  gng.c
 *  Claster
 *
 *  Created by Naoyuki Kubota on 12/05/18.
 *  Copyright 2012 首都大学東京. All rights reserved.
 *
 */


#include "gng.hpp"
//extern "C"{
#include "rnd.h"
#include "malloc.h"
#include "projection.h"
#include "picoCamera.h"
//};
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <GLUT/glut.h>
#include <OpenGL/gl.h>

struct gng *ocnet = NULL;
struct gng *intnet = NULL;
struct gng *init_gng()
{
    int i,j,k;
    struct gng *net=NULL;
    if(net == NULL)
        net = (struct gng *)malloc(sizeof (struct gng));
    
    for (j = 0; j < DIM; j++) {
        net->weight[j] = 1.0;
    }
    net->rung_num = 0;
    for(i = 0; i < RN; i++){
        net->rung_ct[i] = 0;
        net->rung_age[i] = 0;
        for (j = 0; j < DIM; j++) {
            net->rung_node[i][0][j] = 0;
            net->rung_node[i][1][j] = 0;
            net->rung_eval[i][j] = 0;
        }
        for (j = 0; j < GNGN; j++) {
            net->rung_nodeCon[i][j] = 0;
            net->rung_node_list[i][j] = -1;
        }
        
        net->rung_affordance_ct[i] = 0;
        for(j = 0; j < RN; j++){
            net->rung_affordance_posi[i][j] = 0;
            for (k = 0; k < DIM; k++)
                net->rung_affordance_point[i][j][k];
        }
    }
    
    for(i=0;i<GNGN;i++){
        for(j=0;j<DIM;j++){
            net->node[i][j] = 1 * rnd();
            net->prenode[i][j] = net->node[i][j];
            net->delta[i][j] = 0;
        }
        net->triangle[i][0] = 0;
        net->triangle[i][1] = 0;
        net->triangle[i][2] = 0;
        net->triangle[i][3] = 0;
        net->triangle[i][4] = 0;
        net->triangle[i][5] = -1;
        
        net->normTriangle[i][0] = 0;
        net->normTriangle[i][1] = 0;
        net->normTriangle[i][2] = 0;
        net->normTriangle[i][3] = 0;
        net->strength[i] = 0.25;
        
        net->gng_err[i] = 0;
        net->gng_u[i] = 0.0;
        net->movedis[i] = 0.0;
        net->safedeg[i] = 0.0;
        net->edge_dimension[i] = 0;
        
        
        
        for(j=0;j<GNGN;j++){
            net->edge[i][j] = 0;
            net->edgeTriangle[i][j] = 0;
            net->age[i][j] = 0;
        }
    }
    
    for(i=0;i<2;i++)
        for(j=0;j<2;j++){
            if(i != j)
                net->edge[i][j] = 1;
        }
    net->node_n = 2;
    
    net->sigma = 20.0;
    net->cog[0] = 0.0;
    net->cog[1] = 0.0;
    net->cog[2] = 1.0;
    
//    net->parent = NULL;
//    net->child = NULL;
    net->layer = 0;
    net->triangle_n = 0;
    net->intentional_n = 0;
    net->intNode_n = 0;
    net->square_n = 0;
    net->K = 10000;
    net->udrate = 0.001;
    return net;
}

void connect_colordata(struct gng *net, int color[][3], int color_cluster[][3], int dmax)
{
    int i,j,k,l,c;
    int ave_c[CN][3];
    int dis,min_dis;
    int min_n;
    
    c =net->cluster_ct;
    for(i=0;i<c;i++){
        for(l=0;l<3;l++)
            ave_c[i][l] = 0;
        for(j=0;j<net->cluster_num[i];j++){
            k = net->cluster[i][j];
            for(l=0;l<3;l++)
                ave_c[i][l] += (int)net->node[k][l];
        }
        for(l=0;l<3;l++)
            ave_c[i][l] /= net->cluster_num[i];
    }
    
    for(i=0;i<dmax;i++){
        min_dis = 255*255*255;
        for(j=0;j<c;j++){
            dis = 0;
            for(l=0;l<3;l++)
                dis += (ave_c[j][l] - color[i][l])*(ave_c[j][l] - color[i][l]);
            if(min_dis > dis){
                min_dis = dis;
                min_n = j;
            }
        }
        for(l=0;l<3;l++)
            color_cluster[i][l] = ave_c[min_n][l];
    }
}

void connect_distancedata(struct gng *net, double v[][DIM], int distance_cluster[], int dmax, int dim_s, int dim_f)
{
    int i,j,k,l,c,n;
    double dis,min_dis;
    int min_n;
    
    c =net->cluster_ct;
    
    for(i=0;i<dmax;i++){
        min_dis = 1000000;
        for(j=0;j<c;j++){
            for(n=0;n<net->cluster_num[j];n++){
                k = net->cluster[j][n];
                dis = 0;
                for(l=dim_s;l<dim_f;l++)
                    dis += (net->node[k][l] - v[i][l])*(net->node[k][l] - v[i][l]);
                if(min_dis > dis){
                    min_dis = dis;
                    min_n = j;
                }
            }
        }
        distance_cluster[i] = min_n;
    }
}

void gng_triangle_search(struct gng *net)
{
    int i,j,k,l;
    int ct = 0, cn = 0;
    for(i=0;i<net->node_n;i++){
        for(j=i+1;j<net->node_n;j++){
            if((net->edge[i][j] == 1 || net->edgeTriangle[i][j] == 1) && i!=j ){
            for(k=j+1;k<net->node_n;k++){
                if(net->node[i][0] == 0 && net->node[i][1] == 0 && net->node[i][2] == 0) continue;
                if(net->node[j][0] == 0 && net->node[j][1] == 0 && net->node[j][2] == 0) continue;
                if(net->node[k][0] == 0 && net->node[k][1] == 0 && net->node[k][2] == 0) continue;
                if(i > GNGN || j > GNGN || k > GNGN){
                    printf("%d,%d,%d\n",i,j,k);
                    continue;
                }
                
                if((net->edge[i][j] == 1 || net->edgeTriangle[i][j] == 1) && (net->edge[i][k] == 1 || net->edgeTriangle[i][k] == 1) && (net->edge[k][j] == 1 || net->edgeTriangle[k][j] == 1) && i!=j && j !=k && k != i){
                    net->triangle[ct][0] = i;
                    net->triangle[ct][1] = j;
                    net->triangle[ct][2] = k;
                    ct++;
                }
            }
        }
        }
    }
    
    net->triangle_n = ct;
//    for(i=0;i<net->node_n;i++){
//        for(j=0;j<net->node_n;j++){
//            if(net->edge[i][j] == 1 && net->edge[j][i] == 1 && (net->node[i][2] + net->node[j][2])/2 < 700){
//                for(k = 0; k < net->triangle_n; k++){
//                    cn = 0;
//                    for(l = 0; l < 3; l++){
//                        if(net->triangle[k][l] == j || net->triangle[k][l] == i){
//                            cn++;
//                        }
//                    }
//                    if(cn >= 2){
//                        k = net->triangle_n;
//                    }
//                }
//                if(cn < 2){
//                    ct++;
//                    net->triangle_n = ct;
//                    k = ct;
//                    net->triangle[ct][0] = i;
//                    net->triangle[ct][1] = j;
//                    net->triangle[ct][2] = i;
//                    if(ct >= 5000){
//                        i = net->node_n;
//                        j = net->node_n;
//                        k =  net->triangle_n;
//                    }
//                }
//            }
//        }
//    }
    net->triangle_n = ct;
    if(net->triangle_n > 10000){
        printf("test");
    }if(net->triangle_n <= 0){
        printf("test");
    }
}

void search_square(struct gng *net)
{
    int a,b,c,d,i,j;
    int ct = 0;
//    for(i=0;i<GNGN;i++){
//        for(j=0;j<GNGN;j++){
//            net->edgeTriangle[i][j] = 0;
//        }
//    }
    for(a=0;a<net->node_n;a++){
        for(b=0;b<net->node_n;b++){
            if((net->edge[a][b] == 1 && net->edgeTriangle[a][b] == 0)){
            for(c=0;c<net->node_n;c++){
                if((net->edge[a][b] == 1 && net->edgeTriangle[a][b] == 0) && (net->edge[a][c] == 1 && net->edgeTriangle[a][c] == 0) && net->edge[c][b] == 0 && net->edgeTriangle[c][b] == 0 && a != b && b != c && c != a){
                    for(d=0;d<net->node_n;d++){
                        if((net->edge[d][b] == 1  && net->edgeTriangle[d][b] == 0) && (net->edge[d][c] == 1 && net->edgeTriangle[d][c] == 0) && (net->edge[d][a] == 0) && net->edgeTriangle[d][a] == 0 && a != d){
                                net->edgeTriangle[d][a] = 1;
                                net->edgeTriangle[a][d] = 1;
//                                net->edge[d][a] = 1;
//                                net->edge[a][d] = 1;
//                                net->age[d][a] = 0;
//                                net->age[a][d] = 0;
                                ct++;
                        }
                    }
                }
            }
        }
        }
    }
    net->square_n = ct;
    if(net->square_n > 100000){
        printf("test");
    }
}

//void search_pentagon(struct gng *net)
//{
//    int i, j, a,b,c,d,e,o;
//    int ct = 0, cr = 0, cp = 0;
//
//
//    for(a=0;a<net->node_n;a++){
//        for(b=0;b<net->node_n;b++){
//            for(c=0;c<net->node_n;c++){
//                if(net->edge[a][b] == 1 && net->edge[b][c] == 1 && net->edge[c][a] == 0 && net->edgeTriangle[c][a] == 0 && a != b && b != c && c != a){
//                    cp = 0;
//                    for(d=0;d<net->node_n;d++){
//                        if(net->edge[d][c] == 1 && net->edge[d][b] == 0 && net->edge[d][a] == 0 && net->edgeTriangle[d][b] == 0 && net->edgeTriangle[d][a] == 0 && d != a && d != b && d != c){
//                            cp++;
//                            cr = 0;
//                            for(e=0;e<net->node_n;e++){
//                                if(net->edge[e][d] == 1 && net->edge[e][a] == 1 && net->edge[e][b] == 0 && net->edge[e][c] == 0 && e != a && e != b && e != c && e != d){
//                                    cr++;
//                                }
//                            }
//
//                        }
//                    }
//
//                    if(cr == 1 && cp == 1){
//                        net->edgeTriangle[d][a] = 1;
//                        net->edgeTriangle[a][d] = 1;
//                        net->edgeTriangle[c][a] = 1;
//                        net->edgeTriangle[a][c] = 1;
//
//                        ct++;
//                    }
//                }
//            }
//        }
//    }
//}
void search_pentagon(struct gng *net)
{
    int i, j, a,b,c,d,e,f,o;
    int ct = 0, cr = 0, cp = 0;
    
    
    for(a=0;a<net->node_n;a++){
        for(b=0;b<net->node_n;b++){
            if(net->edge[a][b] == 1 && a != b){
            for(c=0;c<net->node_n;c++){
                if(net->edge[a][b] == 1 && net->edge[b][c] == 1 && net->edge[c][a] == 0 && net->edgeTriangle[c][a] == 0 && a != b && b != c && c != a){
                    for(d=0;d<net->node_n;d++){
                        if(net->edge[d][c] == 1 && net->edge[d][b] == 0 && net->edgeTriangle[d][b] == 0 && net->edge[d][a] == 0  && net->edgeTriangle[d][a] == 0 && d != a && d != b && d != c){
                            for(e=0;e<net->node_n;e++){
                                if(net->edge[e][d] == 1 && net->edge[e][a] == 1 && net->edge[e][b] == 0  && net->edgeTriangle[e][b] == 0&&  net->edge[e][c] == 0  && net->edgeTriangle[e][c] == 0 && e != a && e != b && e != c && e != d){
//                                    glLineWidth(1);
                                    int F[5] = {a,b,c,d,e};
                                    for(f=0;f<net->node_n;f++){
                                        cp = 0;
                                        for(i = 0; i < 5; i++){
                                        if((net->edge[f][F[i]] == 1 || net->edgeTriangle[f][F[i]] == 1 ) &&
                                            f!=a && f!=b  && f!=c && f!=d && f!=e)
                                            cp ++;
                                        }
                                        if(cp > 3) f = net->node_n;
                                    }
                                    if(cp < 4){
                                        net->edgeTriangle[d][a] = 1;
                                        net->edgeTriangle[a][d] = 1;
                                        net->edgeTriangle[c][a] = 1;
                                        net->edgeTriangle[a][c] = 1;
//
                                    }
                                }
                            }
                            
                        }
                    }
                    
                }
            }
            }
        }
    }
}


void search_hexagon(struct gng *net)
{
    int a,b,c,d,e,f,o;
    int ct = 0;
    for(a=0;a<net->node_n;a++){
        for(b=0;b<net->node_n;b++){
            for(c=0;c<net->node_n;c++){
                if(net->edge[a][b] == 1 && net->edge[a][c] == 1 && net->edge[c][b] == 0 && a != b && b != c && c != a){
                    for(d=0;d<net->node_n;d++){
                        for(e=0;e<net->node_n;e++){
                            if(net->edge[d][b] == 1 && net->edge[e][c] == 1 && net->edge[e][d] == 0 && d != e){
                                for(f=0;f<net->node_n;f++){
                                    if(net->edge[e][f] == 1 && net->edge[d][f] == 1){
                                        if(net->edge[a][f] == 0 && net->edge[b][f] == 0 && net->edge[c][f] == 0 && f != a && f != b && f != c){
                                            if(net->edge[d][a] == 0 && net->edge[d][c] == 0 && net->edge[e][b] == 0 && net->edge[e][a] == 0 && d != a && e != a){
                                                ct = 0;
                                                for(o=0;o<net->node_n;o++)
                                                    if(net->edge[o][a] == 1 && net->edge[o][b] == 1 && net->edge[o][c] == 1 && net->edge[o][d] == 1 && net->edge[o][e] == 1 && net->edge[o][f] == 1)
                                                        ct++;
                                                if(ct == 0){
                                                    net->edge[d][a] = 1;
                                                    net->edge[a][d] = 1;
                                                    net->edge[e][a] = 1;
                                                    net->edge[a][e] = 1;
                                                    net->edge[f][a] = 1;
                                                    net->edge[a][f] = 1;
                                                    
                                                    net->age[d][a] = 0;
                                                    net->age[a][d] = 0;
                                                    net->age[e][a] = 0;
                                                    net->age[a][e] = 0;
                                                    net->age[f][a] = 0;
                                                    net->age[a][f] = 0;
                                                    
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
double getTriangleArea(float v1[], float v2[], float v3[]){
    float a,b,c,s,L;
    a = norm(v1, 3);
    b = norm(v2, 3);
    c = norm(v3, 3);
    s = 0.5 * (a + b + c);
    
    L = sqrt(s * (s - a) * (s - b) * (s - c));
    
    return L;
}
void normal_vector_triangulation(struct gng *net){
    int i=0, a = 0;
    float *vu, v[3], v1[3], v2[3], v3[3], *v_0, *v_1, r, v_r[3];
//    net->intNode_n=0;
    if(net->triangle_n > 10000){
        printf("test");
    }
    int num = net->triangle_n;
    for(a = 0; a < num; a++){
        if(net->triangle[a][0] != net->triangle[a][2]){
            for(i=0; i<3; i++){
                v1[i] = net->node[net->triangle[a][0]][i] - net->node[net->triangle[a][1]][i];
                v2[i] = net->node[net->triangle[a][0]][i] - net->node[net->triangle[a][2]][i];
                v3[i] = net->node[net->triangle[a][1]][i] - net->node[net->triangle[a][2]][i];
                v_r[i] = net->node[net->triangle[a][0]][i];
            }
            v[0] = v1[1]*v2[2] - v1[2]*v2[1];
            v[1] = v1[2]*v2[0] - v1[0]*v2[2];
            v[2] = v1[0]*v2[1] - v1[1]*v2[0];
            
            net->triangle_area[a] = getTriangleArea(v1,v2,v3);
            
            vu = vectorUnit(v);
            v_0 = centerPointTriangle(net->node[net->triangle[a][0]], net->node[net->triangle[a][1]], net->node[net->triangle[a][2]]);
        }else{
//            float *v0 = (float *) malloc(sizeof (float) * 3);
//            v0[0] = (float)net->node[net->triangle[a][0]][0];
//            v0[1] = (float)net->node[net->triangle[a][0]][1];
//            v0[2] = (float)net->node[net->triangle[a][0]][2];
//
//            for(i=0; i<3; i++){
//                v1[i] = net->node[net->triangle[a][0]][i] - net->node[net->triangle[a][1]][i];
//            }
//            float A = M_PI/2;
//            v2[0] = v1[0] * cos(A) - v1[1] * sin(A);
//            v2[1] = v1[0] * sin(A) + v1[1] * cos(A);
//            v2[2] = 0;
//            v_0 = vectorSubtraction(v0, vectorScale(0.5,v1,3), 3);
//            for(i=0; i<3; i++){
//                v[i] = v1[i] - v2[i];
//                v_r[i] = fmax(v1[i], v2[i]);
//            }
//
//            v[0] = v1[1]*v2[2] - v1[2]*v2[1];
//            v[1] = v1[2]*v2[0] - v1[0]*v2[2];
//            v[2] = v1[0]*v2[1] - v1[1]*v2[0];
//
//            net->triangle_area[a] = 0;//getTriangleArea(v1,v2,v1);
//            vu = vectorUnit(v);
        }
        float scale;
        float va = vu[0]*v_0[0] + vu[1]*v_0[1] + vu[2]*v_0[2];
        float vt = va/(norm(vu, 3) * norm(v_0, 3));
        if(vt > 0) scale = -40;
        else scale = 40;
        
        float vd = abs(vu[1]);
        net->normTriangle[a][3] = (double)(vd/(norm(vu, 3) * 1.0));
        
        r = norm(vectorSubtraction(v_r, v_0, 3), 3);
        float *n = vectorScale(scale, vu, 3);
//        v_1 = vectorAdd(v_0, n, 3);
        
//        net->triangle[a][3] = r;
//
//        net->triangle[a][4] = 0;
//        if(v_0[0] < 150 && v_0[0] > -150 && v_0[2] < 2000 ){
//            if(abs(vu[2]) > abs(vu[0]) && abs(vu[2]) > abs(vu[1])){
//                net->triangle[a][4] = 1;
//                net->intNode_n++;
//            }
//        }
        net->normTriangle[a][0] = (double)n[0];
        net->normTriangle[a][1] = (double)n[1];
        net->normTriangle[a][2] = (double)n[2];
        
//        if(net->triangle[a][3] == 1){
//
//            glColor3f(1.0f, 0.0f, 1.0f);
////            Arrow(v_0[0], v_0[1], v_0[2], v_1[0], v_1[1], v_1[2], 2);
//        }
//        else if(net->normTriangle[a][3] > 0.9){
//
//            glColor3f(0.0f, 1.0f, 0.0f);
////            Arrow(v_0[0], v_0[1], v_0[2], v_1[0], v_1[1], v_1[2], 2);
//        }
//        else{
//            glColor3f(1.0f, 1.0f, 0.0f);
////            Arrow(v_0[0], v_0[1], v_0[2], v_1[0], v_1[1], v_1[2], 2);
//
//        }
        free(vu);
        free(v_0);
        free(n);
//        free(v_1);
    }
    if(net->triangle_n <= 0){
        printf("test");
        net->triangle_n = num;
    }
}
void intentionalRawData(struct gng *net, double P[153216][DIM]){
    int a=0, j=0, i=0, ct=0, cv=0;
    float v[3];
    int A = net->intNode_n;
    if(A > 0){
        float *r = (float *) malloc(sizeof (float) * A);

        float **v_0=(float **) malloc(A*sizeof(float *));
        for(int i=0;i<3;i++)
            v_0[i]=(float *) malloc(3*sizeof(float));
        
        for(a=0; a<net->triangle_n; a++){
            if(net->triangle[a][4] == 1){
                r[cv] = 1.1*(net->triangle[a][3]);
                v_0[cv] = centerPointTriangle(net->node[net->triangle[a][0]], net->node[net->triangle[a][1]], net->node[net->triangle[a][2]]);
                cv++;
            }
        }
    //
        for(i=0; i<153216; i++){
            for(j=0; j<cv; j++){
                v[0] = P[i][0] - v_0[j][0];
                v[1] = P[i][1] - v_0[j][1];
                v[2] = P[i][2] - v_0[j][2];
                if(r[j] > norm(v,3)){
                    dataCamInt[ct][0] = P[i][0];
                    dataCamInt[ct][1] = P[i][1];
                    dataCamInt[ct][2] = P[i][2];
                    ct++;
                    j = cv;
                }
            }
        }
    }
    net->intentional_n = ct;
}
void gng_IntentionNodeTransfer(struct gng *net, struct gng *net2){
    int a=0, i, j, k;
    net2->node_n = 0;
    for(a=0; a<net->triangle_n; a++){
        if(net->triangle[a][4] == 1){
            for(i =0; i<3; i++){
                int sim = 0;
                for(j=0; j<a; j++){
                    for(k =0; k<3; k++){
                    }
                }
                if(sim == 0){
                    net2->node_n++;
                }
            }
        }
    }
}

clock_t start, end;
double cpu_time_used;

void gng_triangulation(struct gng *net)
{
    int i, j;
    for(i=0;i<GNGN;i++){
        for(j=0;j<GNGN;j++){
            net->edgeTriangle[i][j] = 0;
        }
    }
//    start = clock();
//    search_pentagon(net);
//    end = clock();
//    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
//    printf("1: %f\t", cpu_time_used);
    
//    start = clock();
//    search_square(net);
//    end = clock();
//    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
//    printf("2: %f\t", cpu_time_used);
    
    start = clock();
    gng_triangle_search(net);
    end = clock();
    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    printf("3: %f\t", cpu_time_used);
    
    
    start = clock();
    normal_vector_triangulation(net);
    end = clock();
    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    printf("4: %f\t", cpu_time_used);
}
void gng_classification(struct gng *net)
{
    int i,j,k,n;
    int sflag=0;
    int flag[GNGN];
    int c=0;
    int c_n=0;
    int sum=0;
    for(i=0;i<net->node_n;i++)
        flag[i] = 0;
    
    net->cluster[c][c_n] = 0;
    flag[0] = 1;
    c_n++;
    
    while (sum < net->node_n){
        for(i=0;i<c_n;i++){
            n=net->cluster[c][i];
            for(j=0;j<net->node_n;j++){
                if(n != j && net->edge[n][j] == 1){
                    sflag = 0;
                    for(k=0;k<c_n;k++)
                        if(j == net->cluster[c][k])
                            sflag = 1;
                    
                    if(sflag == 0){
                        net->cluster[c][c_n] = j;
                        flag[j] = 1;
                        c_n++;
                    }
                }
            }
        }
        sum+=c_n;
        net->cluster_num[c] = c_n;
        c_n = 0;
        c++;
        for(i=0;i<net->node_n;i++){
            if(flag[i] == 0){
                net->cluster[c][c_n] = i;
                flag[i] = 1;
                c_n++;
                break;
            }
        }
    }
    
    net->cluster_ct = c;
}

int search_maxMD(struct gng *net)
{
    int i,j;
    int sum=0;
    double dis;
    double maxdis= 0.0;
    int max_n=0;
    for(i=0;i<net->node_n;i++){
        sum=1;
        dis = net->movedis[i];
        //        for(j=0;j<net->node_n;j++){
        //
        //            if(net->edge[i][j] == 1){
        //                dis += net->movedis[i];
        //                sum++;
        //            }
        //        }
        dis /= (double)sum;
        if(maxdis < dis){
            maxdis = dis;
            max_n = i;
        }
    }
    
    return max_n;
}

int search_maxCurvature(struct gng *net)
{
    int i;
    double dis = 0;
    double maxdis= 0.0;
    int max_n=0;
    for(i=0;i<net->node_n;i++){
        dis = net->node[i][9];
        if(maxdis < dis){
            maxdis = dis;
            max_n = i;
        }
    }
    
    return max_n;
}

int search_maxSafedeg(struct gng *net)
{
    int i;
    double dis;
    double maxdis= 0.0;
    int max_n=0;
    for(i=0;i<net->node_n;i++){
        dis = net->safedeg[i];
        if(maxdis < dis){
            maxdis = dis;
            max_n = i;
        }
    }
    
    return max_n;
}

void calc_safe_area(struct gng *net)
{
    int i,j,k;
    double inner;
    double dis;
    double vec1[3],vec2[3];
    
    for(i=0;i<net->node_n;i++){
        inner = 0.0;
        dis = 0.0;
        for(k=0;k<3;k++){
            vec1[k] = net->node[i][k+6];
            dis += net->node[i][k]*net->node[i][k];
        }
        
        for(j=0;j<net->node_n;j++){
            if(net->edge[i][j] == 1){
                for(k=0;k<3;k++){
                    vec2[k] = net->node[j][k+6];
                }
                inner += fabs(calc_inner(vec1, vec2, 3));
            }
        }
        net->safedeg[i] = exp(-(dis-4.0)*(dis-4.0)/121.0)*inner;
    }
}

void label_sort(struct gng *net)
{
    int i,j,k;
    int ct = 0;
    
    for(i=0;i<net->label_num;i++){
        if(net->label_ct[i] == 0){
            j = i+1;
            while (net->label_ct[j] == 0) {
                j++;
                if(j == net->label_num) break;
            }
            
            if(j == net->label_num) break;
            
            net->label_ct[i] = net->label_ct[j];
            for(k=0;k<3;k++){
                net->label_node[i][k] = net->label_node[j][k];
                net->label_node[i][k+6] = net->label_node[j][k+6];
            }
            
            for(k=0;k<net->label_ct[j];k++){
                net->label[net->label_list[j][k]] = i;
                net->label_list[i][k] = net->label_list[j][k];
            }
            net->label_ct[j] = 0;
        }
    }
    
    k = 0;
    for(i=0;i<net->label_num;i++){
        if(net->label_ct[i] != 0){
            k++;
        }
    }
    net->label_num = k;
}

int label_to_data(struct gng *net, double v[])
{
    int i,j;
    int s1;
    double mindis, dis;
    
    dis = 0.0;
    for(i=0;i<3;i++)
        dis += (net->node[0][i] - v[i])*(net->node[0][i] - v[i]);
    
    mindis = dis;
    s1 = 0;
    
    for (i=1;i<net->node_n;i++){
        dis = 0;
        
        for(j=0;j<3;j++)
            dis += (net->node[i][j] - v[j])*(net->node[i][j] - v[j]);
        if(dis < mindis){
            mindis = dis;
            s1 = i;
        }
    }
    
    return net->label[s1];
}

void calc_surface_label(struct gng *net)
{
    int i,j,k,l,c;
    double inner;
    double dis;
    double vec1[3],vec2[3];
    double cur1,cur2;
    int label_num = 0;
    int tct = 0;
    const double th = cos(10.0/180.0*M_PI);
    
    for(i=0;i<net->node_n;i++){
        net->label[i] = -1;
        if(i < net->label_num){
            for(j=0;j<net->label_num;j++){
                net->label_edge[i][j] = 0;
            }
        }
    }
    
    for(i=0;i<net->node_n;i++){
        if(net->label[i] != -1) continue;
        
        net->label[i] = label_num;
        inner = 0.0;
        dis = 0.0;
        net->label_list[label_num][0] = i;
        net->label_ct[label_num] = 1;
        for(k=0;k<3;k++){
            vec1[k] = net->node[i][k+6];
            net->label_node[label_num][k] = net->node[i][k];
            net->label_node[label_num][k+6] = vec1[k];
            dis += net->node[i][k]*net->node[i][k];
        }
        cur1 = net->node[i][9];
        for(j=0;j<net->node_n;j++){
            
            if(net->label[j] != -1){
                if(net->edge[i][j] == 1){
                    net->label_edge[net->label[i]][net->label[j]] = 1;
                    net->label_edge[net->label[j]][net->label[i]] = 1;
                }
                continue;
            }
            
            if(net->edge[i][j] == 1){
                for(k=0;k<3;k++){
                    vec2[k] = net->node[j][k+6];
                }
                cur2 = net->node[j][9];
                inner = calc_inner(vec1, vec2, 3);
                if(fabs(inner) > th || (cur1 == -10 && cur2 == -10)){
                    net->label[j] = label_num;
                    net->label_list[label_num][net->label_ct[label_num]] = j;
                    for(k=0;k<3;k++){
                        net->label_node[label_num][k] += net->node[j][k];
                        if(inner > 0)
                            net->label_node[label_num][k+6] += vec2[k];
                        else
                            net->label_node[label_num][k+6] += -1.0*vec2[k];
                    }
                    net->label_ct[label_num]++;
                }
            }
        }
        
        l = 0;
        while(net->label_ct[label_num] != l){
            c = net->label_list[label_num][l];
            for(k=0;k<3;k++) vec1[k] = net->node[c][k+6];
            cur2 = net->node[c][9];
            for(j=0;j<net->node_n;j++){
                if(net->label[j] != -1 || net->label[j] == label_num){
                    if(net->edge[i][j] == 1){
                        net->label_edge[net->label[i]][net->label[j]] = 1;
                        net->label_edge[net->label[j]][net->label[i]] = 1;
                    }
                    continue;
                }
                
                if(net->edge[c][j] == 1){
                    for(k=0;k<3;k++){
                        vec2[k] = net->node[j][k+6];
                    }
                    cur2 = net->node[j][9];
                    inner = calc_inner(vec1, vec2, 3);
                    if(fabs(inner) > th || (cur1 == -10 && cur2 == -10)){
                        net->label[j] = label_num;
                        net->label_list[label_num][net->label_ct[label_num]] = j;
                        for(k=0;k<3;k++){
                            net->label_node[label_num][k] += net->node[j][k];
                            if(inner > 0)
                                net->label_node[label_num][k+6] += vec2[k];
                            else
                                net->label_node[label_num][k+6] += -1.0*vec2[k];
                        }
                        net->label_ct[label_num]++;
                    }
                }
            }
            
            l++;
        }
        for (k = 0; k < 3; k++) {
            net->label_node[label_num][k] /= (double)net->label_ct[label_num];
            net->label_node[label_num][k + 6] /= (double)net->label_ct[label_num];
        }
        label_num++;
    }
    net->label_num = label_num;
    
    label_sort(net);
}

void calc_surface_label2(struct gng *net)
{
    int i,j,k,l,c;
    double inner;
    double dis;
    double vec1[3],vec2[3];
    int label_num = 0;
    int tct = 0;
    const double th = cos(20.0/180.0*M_PI);
    
    for(i=0;i<net->node_n;i++){
        net->label[i] = -1;
        if(i < net->label_num){
            for(j=0;j<net->label_num;j++){
                net->label_edge[i][j] = 0;
            }
        }
    }
    for(i=0;i<net->node_n;i++){
        if(net->label[i] != -1) continue;
        
        net->label[i] = label_num;
        inner = 0.0;
        dis = 0.0;
        net->label_list[label_num][0] = i;
        net->label_ct[label_num] = 1;
        for(k=0;k<3;k++){
            vec1[k] = net->node[i][k+6];
            net->label_node[label_num][k] = net->node[i][k];
            net->label_node[label_num][k+6] = vec1[k];
            dis += net->node[i][k]*net->node[i][k];
        }
        for(j=0;j<net->node_n;j++){
            
            if(net->label[j] != -1){
                if(net->edge[i][j] == 1){
                    net->label_edge[net->label[i]][net->label[j]] = 1;
                    net->label_edge[net->label[j]][net->label[i]] = 1;
                }
                continue;
            }
            
            if(net->edge[i][j] == 1){
                for(k=0;k<3;k++){
                    vec2[k] = net->node[j][k+6];
                }
                
                inner = calc_inner(vec1, vec2, 3);
                if(fabs(inner) > th){
                    net->label[j] = label_num;
                    net->label_list[label_num][net->label_ct[label_num]] = j;
                    for(k=0;k<3;k++){
                        net->label_node[label_num][k] += net->node[j][k];
                        if(inner > 0)
                            net->label_node[label_num][k+6] += vec2[k];
                        else
                            net->label_node[label_num][k+6] += -1.0*vec2[k];
                    }
                    net->label_ct[label_num]++;
                }
            }
        }
        
        l = 0;
        while(net->label_ct[label_num] != l){
            c = net->label_list[label_num][l];
            for(j=0;j<net->node_n;j++){
                if(net->label[j] != -1 || net->label[j] == label_num){
                    if(net->edge[i][j] == 1){
                        net->label_edge[net->label[i]][net->label[j]] = 1;
                        net->label_edge[net->label[j]][net->label[i]] = 1;
                    }
                    continue;
                }
                
                if(net->edge[c][j] == 1){
                    for(k=0;k<3;k++){
                        vec2[k] = net->node[j][k+6];
                    }
                    
                    inner = calc_inner(vec1, vec2, 3);
                    if(fabs(inner) > th){
                        net->label[j] = label_num;
                        net->label_list[label_num][net->label_ct[label_num]] = j;
                        for(k=0;k<3;k++){
                            net->label_node[label_num][k] += net->node[j][k];
                            if(inner > 0)
                                net->label_node[label_num][k+6] += vec2[k];
                            else
                                net->label_node[label_num][k+6] += -1.0*vec2[k];
                        }
                        net->label_ct[label_num]++;
                    }
                }
            }
            
            l++;
        }
        
        for(k=0;k<3;k++){
            net->label_node[label_num][k] /= (double)net->label_ct[label_num];
            net->label_node[label_num][k+6] /= (double)net->label_ct[label_num];
        }
        
        label_num++;
    }
    
    net->label_num = label_num;
    
    int list[CN]={0};
    int lct = 0;
    int min_n;
    double mindis;
    for(i=0;i<net->label_num;i++){
        lct = 0;
        for(j=0;j<net->label_num;j++){
            if(i == j || net->label_ct[i] == 0 || net->label_ct[j] == 0) continue;
            if(net->label_edge[i][j] == 1){
                for(k=0;k<3;k++){
                    vec1[k] = net->label_node[i][k+6];
                    vec2[k] = net->label_node[j][k+6];
                }
                inner = calc_inner(vec1, vec2, 3);
                
                if(fabs(inner) > th){
                    label_num = net->label_ct[i];
                    for(k=0;k<net->label_ct[j];k++){
                        net->label[net->label_list[j][k]] = i;
                        net->label_list[i][label_num] = net->label_list[j][k];
                        label_num++;
                    }
                    
                    for(k=0;k<3;k++){
                        net->label_node[i][k] = (double)net->label_ct[i]*net->label_node[i][k] + (double)net->label_ct[j]*net->label_node[j][k];
                        if(inner > 0)
                            net->label_node[label_num][k+6] += vec2[k];
                        else
                            net->label_node[label_num][k+6] += -1.0*vec2[k];
                    }
                    
                    net->label_ct[i] = label_num;
                    net->label_ct[j] = 0;
                    for(k=0;k<3;k++){
                        net->label_node[i][k] /= (double)net->label_ct[i];
                        net->label_node[i][k+6] /= 2.0;
                    }
                    tct++;
                }else if(net->label_ct[i] == 1){
                    list[lct] = j;
                    lct++;
                }
                
            }
        }
        
        if(net->label_ct[i] == 1 && lct != 0){
            min_n = 0;
            mindis = (net->label_node[i][0] - net->label_node[list[0]][0])*(net->label_node[i][0] - net->label_node[list[0]][0]);
            for(j=1;j<3;j++) mindis += (net->label_node[i][j] - net->label_node[list[0]][j])*(net->label_node[i][j] - net->label_node[list[0]][j]);
            for(k=1;k<lct;k++){
                dis = (net->label_node[i][0] - net->label_node[list[k]][0])*(net->label_node[i][0] - net->label_node[list[k]][0]);
                for(j=1;j<3;j++) dis += (net->label_node[i][j] - net->label_node[list[k]][j])*(net->label_node[i][j] - net->label_node[list[k]][j]);
                if(mindis > dis){
                    mindis = dis;
                    min_n = list[k];
                }
            }
            
            label_num = net->label_ct[min_n];
            for(k=0;k<net->label_ct[i];k++){
                net->label[net->label_list[i][k]] = min_n;
                net->label_list[min_n][label_num] = net->label_list[i][k];
                label_num++;
            }
            
            for(k=0;k<3;k++){
                net->label_node[min_n][k] = (double)net->label_ct[i]*net->label_node[i][k] + (double)net->label_ct[min_n]*net->label_node[min_n][k];
            }
            
            net->label_ct[min_n] = label_num;
            net->label_ct[i] = 0;
            
            for(k=0;k<3;k++){
                net->label_node[min_n][k] /= (double)net->label_ct[min_n];
            }
            
            tct++;
        }
        
    }
    
    label_sort(net);
}


void discount_err_gng(struct gng *net)
{
    int i;
    
    for(i=0;i<net->node_n;i++){
        net->gng_err[i] -= 0.005*net->gng_err[i];
        net->gng_u[i] -= (net->udrate/pow((net->strength[i] * 3) ,1.0))*net->gng_u[i];
        
        if(net->gng_err[i] < 0)
            net->gng_err[i] = 0.0;
        
        if(net->gng_u[i] < 0)
            net->gng_u[i] = 0.0;
        
    }
}

int node_delete(struct gng *net, int i)
{
    int j,k,l,id;
    
    for(id = 0;id<net->rung_num;id++){
        for(j=i;j<net->node_n;j++){
            net->rung_nodeCon[id][j] = net->rung_nodeCon[id][j+1];
        }
        net->rung_nodeCon[id][j+1] = 0;
        net->rung_ct[id] = 0;
        
        for(j=0;j<net->node_n;j++){
            if(net->rung_nodeCon[id][j] != 0){
                net->rung_node_list[id][net->rung_ct[id]] = j;
                net->rung_ct[id]++;
            }
        }
    }
    
    
    for(j=i;j<net->node_n;j++){
        
        net->gng_err[j] = net->gng_err[j+1];
        net->gng_u[j] = net->gng_u[j+1];
        net->movedis[j] = net->movedis[j+1];
        net->wct[j] = net->wct[j+1];
        net->strength[j] = net->strength[j+1];
        
        for(l=0;l<DIM;l++){
            if(l < 3){
                net->dir[j][l] = net->dir[j+1][l];
            }
            net->node[j][l] = net->node[j+1][l];
            net->prenode[j][l] = net->prenode[j+1][l];
        }
        for(k=0;k<net->node_n;k++){
            if(k < i){
                net->age[j][k] = net->age[j+1][k];
                net->age[k][j] = net->age[k][j+1];
                net->edge[j][k] = net->edge[j+1][k];
                net->edge[k][j] = net->edge[k][j+1];
            }else{
                net->age[j][k] = net->age[j+1][k+1];
                net->edge[j][k] = net->edge[j+1][k+1];
            }
        }
    }
    
    for(k=0;k<net->node_n;k++){
        net->age[net->node_n][k] = 0;
        net->age[k][net->node_n] = 0;
        net->edge[net->node_n][k] = 0;
        net->edge[k][net->node_n] = 0;
    }
    
    net->node_n--;
    if (net->node_n <= 0) {
        printf("test");
    }
    return net->node_n;
}


void node_add_gng(struct gng *net, int flag)
{
    int i,j;
    double max_err[2];
    double min_u;
    int u;
    int q;
    int f;
    int r;
    
    max_err[0] = net->gng_err[0];
    q = 0;
    min_u = net->gng_u[0];
    u = 0;
    net->movedis[0] = 0.0;
    
    
    
    for(j=0;j<DIM;j++){
        if(j < 3){
            net->dir[0][j] = net->node[0][j] - net->prenode[0][j];
            net->movedis[0] += net->dir[0][j]*net->dir[0][j];
        }
        net->prenode[0][j] = net->node[0][j];
    }
    
    for (i=1;i<net->node_n;i++){
        net->movedis[i] = 0.0;
        for(j=0;j<DIM;j++){
            if(j < 3){
                net->dir[i][j] = net->node[i][j] - net->prenode[i][j];
                net->movedis[i] += net->dir[i][j]*net->dir[i][j];
            }
            net->prenode[i][j] = net->node[i][j];
        }
        //        if(max_err[0] < net->gng_err[i]*15 && rangeArea(net->node[i][0],net->node[i][1],net->node[i][2],0,30,170,250,250,50)==1){
        //            max_err[0] = net->gng_err[i]*15;
        //            q = i;
        //        }else if(max_err[0] < net->gng_err[i]){
        //            max_err[0] = net->gng_err[i];
        //            q = i;
        //        }
        if(max_err[0] < net->gng_err[i] * pow((4*net->strength[i]),0)){
            max_err[0] = net->gng_err[i] * pow((4*net->strength[i]),0);
            q = i;
        }
        
        if(min_u > net->gng_u[i]){
            min_u = net->gng_u[i];
            u = i;
        }
    }
    max_err[1] = 0;
    f = GNGN;
    for (i=0;i<net->node_n;i++){
        if(net->edge[q][i] == 1 && q != i){
            //            if(net->gng_err[i]*15 > max_err[1] && rangeArea(net->node[i][0],net->node[i][1],net->node[i][2],0,30,170,250,250,50)==1){
            //                max_err[1] = net->gng_err[i]*15;
            //                f = i;
            //            }else if(net->gng_err[i] > max_err[1]){
            //                max_err[1] = net->gng_err[i];
            //                f = i;
            //            }
            if(net->gng_err[i] * pow((4*net->strength[i]),0) > max_err[1]){
                max_err[1] = net->gng_err[i] * pow((4*net->strength[i]),0);
                f = i;
            }
        }
    }
    r = net->node_n;
    for(i=0;i<DIM;i++){
        net->node[r][i] = 0.5*(net->node[q][i] + net->node[f][i]);
        if(i < 3){
            net->prenode[r][i] = net->node[r][i];
            net->dir[r][i] = 0.0;
        }
    }
    net->edge[q][f] = 0;
    net->edge[f][q] = 0;
    net->age[q][f] = 0;
    net->age[f][q] = 0;
    
    net->edge[q][r] = 1;
    net->edge[r][q] = 1;
    net->age[q][r] = 0;
    net->age[r][q] = 0;
    
    net->edge[r][f] = 1;
    net->edge[f][r] = 1;
    net->age[r][f] = 0;
    net->age[f][r] = 0;
    
    net->gng_err[q] -= 0.5*net->gng_err[q];
    net->gng_err[f] -= 0.5*net->gng_err[f];
    
    net->gng_u[q] -= 0.5*net->gng_u[q];
    net->gng_u[f] -= 0.5*net->gng_u[f];
    
    net->gng_err[r] = net->gng_err[q];
    net->gng_u[r] = net->gng_u[q];
    net->node_n = r+1;
    
    gng_calc_strengh(net, r);
    
    //    if(rangeArea(net->node[u][0],net->node[u][1],net->node[u][2],0,30,170,250,250,50)==1)
    //        net->K = 10000;
    //    else
    double K = net->K;// * pow((4*net->strength[u]),0);
    if(max_err[0] > K*min_u && flag == 1){
        node_delete(net, u);
        //        if(net->node[u][1] < -50){
        //            printf("test");
        //        }
        
    }
//    printf("node num = %d\n", net->node_n);
}void node_add_gng2(struct gng *net, int flag)
{
    int i,j;
    double max_err[2];
    double min_u;
    int u;
    int q;
    int f;
    int r;
    
    max_err[0] = net->gng_err[0];
    q = 0;
    min_u = net->gng_u[0];
    u = 0;
    net->movedis[0] = 0.0;
    
    
    
    for(j=0;j<DIM;j++){
        if(j < 3){
            net->dir[0][j] = net->node[0][j] - net->prenode[0][j];
            net->movedis[0] += net->dir[0][j]*net->dir[0][j];
        }
        net->prenode[0][j] = net->node[0][j];
    }
    
    for (i=1;i<net->node_n;i++){
        net->movedis[i] = 0.0;
        for(j=0;j<DIM;j++){
            if(j < 3){
                net->dir[i][j] = net->node[i][j] - net->prenode[i][j];
                net->movedis[i] += net->dir[i][j]*net->dir[i][j];
            }
            net->prenode[i][j] = net->node[i][j];
        }
        //        if(max_err[0] < net->gng_err[i]*15 && rangeArea(net->node[i][0],net->node[i][1],net->node[i][2],0,30,170,250,250,50)==1){
        //            max_err[0] = net->gng_err[i]*15;
        //            q = i;
        //        }else if(max_err[0] < net->gng_err[i]){
        //            max_err[0] = net->gng_err[i];
        //            q = i;
        //        }
        if(max_err[0] < net->gng_err[i] * pow((4*net->strength[i]),4)){
            max_err[0] = net->gng_err[i] * pow((4*net->strength[i]),4);
            q = i;
        }
        
        if(min_u > net->gng_u[i]){
            min_u = net->gng_u[i];
            u = i;
        }
    }
    max_err[1] = 0;
    f = GNGN;
    for (i=0;i<net->node_n;i++){
        if(net->edge[q][i] == 1 && q != i){
            //            if(net->gng_err[i]*15 > max_err[1] && rangeArea(net->node[i][0],net->node[i][1],net->node[i][2],0,30,170,250,250,50)==1){
            //                max_err[1] = net->gng_err[i]*15;
            //                f = i;
            //            }else if(net->gng_err[i] > max_err[1]){
            //                max_err[1] = net->gng_err[i];
            //                f = i;
            //            }
            if(net->gng_err[i] * pow((4*net->strength[i]),4) > max_err[1]){
                max_err[1] = net->gng_err[i] * pow((4*net->strength[i]),4);
                f = i;
            }
        }
    }
    r = net->node_n;
    for(i=0;i<DIM;i++){
        net->node[r][i] = 0.5*(net->node[q][i] + net->node[f][i]);
        if(i < 3){
            net->prenode[r][i] = net->node[r][i];
            net->dir[r][i] = 0.0;
        }
    }
    net->edge[q][f] = 0;
    net->edge[f][q] = 0;
    net->age[q][f] = 0;
    net->age[f][q] = 0;
    
    net->edge[q][r] = 1;
    net->edge[r][q] = 1;
    net->age[q][r] = 0;
    net->age[r][q] = 0;
    
    net->edge[r][f] = 1;
    net->edge[f][r] = 1;
    net->age[r][f] = 0;
    net->age[f][r] = 0;
    
    net->gng_err[q] -= 0.5*net->gng_err[q];
    net->gng_err[f] -= 0.5*net->gng_err[f];
    
    net->gng_u[q] -= 0.5*net->gng_u[q];
    net->gng_u[f] -= 0.5*net->gng_u[f];
    
    net->gng_err[r] = net->gng_err[q];
    net->gng_u[r] = net->gng_u[q];
    net->node_n = r+1;
    
    gng_calc_strengh(net, r);
    
    //    if(rangeArea(net->node[u][0],net->node[u][1],net->node[u][2],0,30,170,250,250,50)==1)
    //        net->K = 10000;
    //    else
    double K = net->K;// * pow((4*net->strength[u]),4);
    if(max_err[0] > K*min_u && flag == 1){
        node_delete(net, u);
        //        if(net->node[u][1] < -50){
        //            printf("test");
        //        }
        
    }
//    printf("node num = %d\n", net->node_n);
}

void node_delete_u(struct gng *net)
{
    int i;
    double max_err;
    double min_u;
    int u;
    int q;
    
    max_err = net->gng_err[0];
    q = 0;
    min_u = net->gng_u[0];
    u = 0;
    
    if (net->node_n <= 0) {
        printf("test");
    }
    for (i=1;i<net->node_n;i++){
        if(max_err < net->gng_err[i]){
            max_err = net->gng_err[i];
            q = i;
        }
        if(min_u > net->gng_u[i]){
            min_u = net->gng_u[i];
            u = i;
        }
    }
//    if(rangeArea(net->node[u][0],net->node[u][1],net->node[u][2],0,30,170,250,250,50)==1)
//        net->K = 10000;
//    else
    double K = net->K;// * pow((4*net->strength[u]),4);
    
    if(max_err > K*min_u){
        node_delete(net, u);
    }
    
    
}


int calc_age(int s1, int s2, struct gng *net)
{
    int i,j;
    int d_flag = 0;
    const int MAX_AGE[4] = {120, 80, 30, 15};
    if(net->edge[s1][s2] != 1){
        net->edge[s1][s2] = 1;
        net->edge[s2][s1] = 1;
    }
    
    net->age[s1][s2] = 0;
    net->age[s2][s1] = 0;
    
    for(i=0;i<net->node_n;i++){
        d_flag = 0;
        if(i != s1){
            if(net->edge[s1][i] == 1 && i != s2){
                net->age[s1][i]++;
                net->age[i][s1]++;
//                if((net->node[s1][1] < -50 && net->node[s1][0] > 50) || (net->node[s1][1] < -50 && net->node[s1][0] > 50)){
//
//                    net->age[s1][i]--;
//                    net->age[i][s1]--;
//                }
                    if(MAX_AGE[net->layer] < net->age[s1][i]){
                        net->age[s1][i] = 0;
                        net->age[i][s1] = 0;
                        net->edge[s1][i] = 0;
                        net->edge[i][s1] = 0;
                        for(j=0;j<net->node_n;j++)
                            if(net->edge[j][i] == 1){
                                d_flag = 1;
                                break;
                            }
                        
                        if(d_flag == 0 && net->node_n > 2){
                            net->node_n = node_delete(net, i);
                            if(s1 > i)
                                s1--;
                            if(s2 > i)
                                s2--;
                            i--;
                        }
                    }
                
            }
        }
    }
    
    return net->node_n;
}

void gng_learn(struct gng *net, int s1, double v[], const double e1, const double e2, int sdim, int edim)
{
    int i,j;
    double d;
    //    d[g1]=(ld[j]-gng[g1][j])*(ld[j]-gng[g1][j])
    //     gng[g1][j]+=r1*(ld[j]-gng[g1][j])*exp(-d[g1]/500.0);
    for(i=sdim;i<edim;i++){
        net->node[s1][i] += e1*(v[i] - net->node[s1][i]);
        if (net->node[s1][i] > 10000 || net->node[s1][i] < -10000) {
            printf("test");
        }
    }
    
    for(i=0;i<net->node_n;i++){
        if(net->edge[s1][i] == 1){
            for (j = sdim; j < edim; j++) {
                net->node[i][j] += e2*(v[j] - net->node[i][j]);
                if (net->node[i][j] > 10000 || net->node[i][j] < -10000) {
                    printf("test");
                }
            }
        }
    }
}
double time_strength = 0;
void gng_calc_strengh(struct gng *net, int s1){
    int i;
    double* v = (double *) malloc(sizeof (double ) * 3);
    int cN = 0;
    v[0]=0;v[1]=0;v[2]=0;
    double va;
//    for(i=0;i<net->node_n;i++){
//        if(i != s1){
//            if(net->edge[s1][i] == 1){
//                double *w = vectorSubtractionD(net->node[s1], net->node[i], 3);
//                double *vv = vectorScaleD(1, v, 3);
////                v = vectorAddD(vv, vectorUnitD(w), 3);
//                cN++;
//                free(w);
//                free(vv);
//            }
//        }
//    }
    
    clock_t start, end;
    double cpu_time_used;

    start = clock();
    
    va = abs(normD(v, 3));
    if(va > 1) va = 1;

    va = 0;
    if(net->node[s1][1] > GROUND_LIMIT && net->node[s1][0] > LEFT_LIMIT && net->node[s1][0] < RIGHT_LIMIT && net->node[s1][2] < LENGTH_LIMIT){
        
        for(i = 0; i < net->sus_object_num; i++){
            if(net->sus_object_list[i][GNGN-1]==2){
                if(rangeArea2(net->node[s1][0],net->node[s1][1],net->node[s1][2], net->sus_object_size[i][0][0]-20, net->sus_object_size[i][0][1]+20,
                              net->sus_object_size[i][1][0]-20, net->sus_object_size[i][1][1]+20,
                              net->sus_object_size[i][2][0]-20, net->sus_object_size[i][2][1]+20) == 1){
//                        va = 0; i = net->sus_object_num;
                }
            }else
                if(net->sus_object_list[i][GNGN-1]==1 && abs(net->sus_object_size[i][1][0] - net->sus_object_size[i][1][1]) > 0){
                    if(rangeArea2(net->node[s1][0],net->node[s1][1],net->node[s1][2], net->sus_object_size[i][0][0]-50, net->sus_object_size[i][0][1]+50,
                                  net->sus_object_size[i][1][0]-50, net->sus_object_size[i][1][1]+50,
                                  net->sus_object_size[i][2][0]-50, net->sus_object_size[i][2][1]+50) == 1){
//                        if(net->node[s1][0] > -300 && net->node[s1][0] < 300 && net->node[s1][2] < 2050 && net->node[s1][1] > -85)
                            va = 1; i = net->sus_object_num;
                }
            }
        }
        
//        if(net->node[s1][0] > -300 && net->node[s1][0] < 300 && net->node[s1][2] < 1550)// && net->node[s1][1] > -85)
//            va = 1; i = net->sus_object_num;
    }
    
    end = clock();
    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    time_strength += cpu_time_used;
//    printf("test2: %f\t", cpu_time_used);
//    if(rangeArea(net->node[s1][0],net->node[s1][1],net->node[s1][2],0,30,170,250,250,50) == 1){
//        va = 1;
//    }else{
//        va = 0;
//    }
//    va = 0;
    net->strength[s1] = 0.35 + 0.95*va;

    free(v);

}

void gng_learn_local_fcm(struct gng *net, int s1, int s2, double v[], double dis[])
{
    int i,j;
    double e1 = 0;
    int cct = 0;
    int *clist = (int *)malloc(sizeof(int)*net->node_n);
    double *mu = (double *)malloc(sizeof(double)*net->node_n);
    const double m = 2;
    const double ETA = 0.2;
    
    if(dis[s1] != 0){
        clist[cct] = s1;
        cct++;
    }else{
        e1 = ETA;
        float wEta = 1;
//        wEta = 1/pow((4 * net->strength[s1]),2);
//        if(net->node[s1][1] < -50 && net->node[s1][0] > 50){
//            wEta = 1;
//        }
//        if(v[1] < -50 && v[0] > 50){
//            wEta = 1;
//        }
        for(i=0;i<DIM;i++){
            net->node[s1][i] += e1*(v[i]-net->node[s1][i])*wEta;
            if (net->node[s1][i] > 10000 || net->node[s1][i] < -10000) {
                printf("test");
            }
        }
        
        free(mu);
        free(clist);
        return;
    }
    
    gng_calc_strengh(net, s1);
    if(dis[s2] != 0){
        clist[cct] = s2;
        cct++;
    }
    
    for(i=0;i<net->node_n;i++){
        if(net->edge[s1][i] == 1 && s1 != i && s2 != i){
            if(dis[i] != 0){
                clist[cct] = i;
                cct++;
            }
        }
    }
    
    for(i=0;i<cct;i++){
        mu[i] = 0.0;
        for(j=0;j<cct;j++){
            mu[i] += pow(dis[clist[i]]/dis[clist[j]], 1.0/(m-1.0));
        }
        mu[i] = 1.0/mu[i];
        e1 = ETA;
        for(j=0;j<m;j++)
            e1 *= mu[i];
        
        
        float wEta = 1;
//        wEta = 1/pow((4 * net->strength[clist[i]]),2);
//        if(net->node[clist[i]][1] < -50 && net->node[clist[i]][0] > 50){
//            wEta = 1;
//        }
//        if(v[1] < -50 && v[0] > 50){
//            wEta = 1;
//        }
        for(j=0;j<DIM;j++)
            net->node[clist[i]][j] += e1*(v[j]-net->node[clist[i]][j]) * wEta;
        
        gng_calc_strengh(net, clist[i]);
    }
    
    free(mu);
    free(clist);
}


int gng_main(struct gng *net, double v[][DIM],int tt, int dmax)
{
    int i=0,j=0,k=0;
    int s1=0,s2=0;
    double mindis=0, mindis2=0, dis[GNGN];
    for (i = 0; i < GNGN; i++) {
        dis[i] = 0;
    }
    static const int ramda[4] = {300, 200 , 100, 25};
    int l[300];
    //int l[ramda[int(0)]];
    
    for(i=0;i<ramda[0];i++){
        l[i] = (int)((double)dmax*rnd());
        if(l[i] >= dmax)
            l[i] = dmax - 1;
        if(l[i] < 0)
            l[i] = 0;
    }
    
    for(k=0;k<ramda[0];k++){
        dis[0] = 0.0;
        for(i=0;i<DIM;i++)
            dis[0] += net->weight[i]*(net->node[0][i] - v[l[k]][i])*(net->node[0][i] - v[l[k]][i]);
        
        mindis = dis[0];
        s1 = 0;
        
        for (i=1;i<net->node_n;i++){
            dis[i] = 0;
            
            for(j=0;j<DIM;j++)
                dis[i] += net->weight[j]*(net->node[i][j] - v[l[k]][j])*(net->node[i][j] - v[l[k]][j]);
            if(dis[i] < mindis){
                mindis = dis[i];
                s1 = i;
            }
        }
        
        if(s1 != 0){
            mindis2 = dis[0];
            s2 = 0;
        }else{
            mindis2 = dis[1];
            s2 = 1;
        }
        for (i=0;i<net->node_n;i++){
            if(dis[i] < mindis2 && i != s1){
                mindis2 = dis[i];
                s2 = i;
            }
        }
        
        net->gng_err[s1] += mindis;
        
        if(net->node_n < GNGN)
            gng_learn(net, s1, v[l[k]], 0.08, 0.008, 0, 3);
        
        net->node_n = calc_age(s1, s2, net);
        discount_err_gng(net);
    }
    
    node_add_gng(net, 1);
    
    return net->node_n;
}

int rangeArea(double x, double y, double z, double x_pos, double y_pos, double z_pos, double x_size, double y_size, double z_size){
    int PASS = 0;
    if(x > x_pos-x_size/2 && x < x_pos+x_size/2)
        if(y > y_pos-y_size/2 && y < y_pos+y_size/2)
            if(z > z_pos-z_size/2 && z < z_pos+z_size/2)
                PASS = 1;
    
    return PASS;
}int rangeArea2(double x, double y, double z, double x_min, double x_max, double y_min, double y_max, double z_min, double z_max){
    int PASS = 0;
    if(x > x_min && x < x_max)
        if(y > y_min && y < y_max)
            if(z > z_min && z < z_max)
                PASS = 1;
    
    return PASS;
}
int last_node = 1;
int gng_main_oct(struct gng *net, double P[153216][DIM], int dmax, int tt, int flag)
{
    int i,j,k;
    int s1 = 0,s2 = 0;
    double mindis = 0, mindis2 = 0, dis[GNGN];
    const int ramda[4] = {500, 200 , 100, 25};
    int l[ramda[net->layer]];
    //int l[3000];
    double **v;
    
    v = malloc2d_double(dmax, DIM);
    
    
    static int in = 0;
    int sus_count=0;
    
    for(j = 0; j < net->sus_object_num; j++){
        if(net->sus_object_list[j][GNGN-1] == 1){
            net->obs_count = sus_count++;
        }
    }
    for(i=0;i<ramda[net->layer];i++){
        l[i] = (int)((double)dmax*rnd());
        in = 0;
        int count = 0;
        while((sus_count > 2 && in != 1 && i %2 != 0) && count < 100000){
            while((P[l[i]][1] < GROUND_LIMIT || P[l[i]][0] < LEFT_LIMIT || P[l[i]][0] > RIGHT_LIMIT || in == 2 || P[l[i]][2] > LENGTH_LIMIT) && count < 100000){
                l[i] = (int)((double)dmax*rnd());
                in = 0;
                count++;
            }
            count++;
            in = 2;
            for(j = 0; j < net->sus_object_num; j++){
                if(net->sus_object_list[j][GNGN-1] == 1 ){
                    if(rangeArea2(P[l[i]][0],P[l[i]][1],P[l[i]][2], net->sus_object_size[j][0][0]-20, net->sus_object_size[j][0][1]+20,
                                  net->sus_object_size[j][1][0]-20, net->sus_object_size[j][1][1]+20,
                                  net->sus_object_size[j][2][0]-20, net->sus_object_size[j][2][1]+20) == 1){
                        j = net->sus_object_num;
                        in = 1;
                    }
                }
            }
        }
//        while(rangeArea(P[l[i]][0],P[l[i]][1],P[l[i]][2],0,0,170,500,500,500) == 0 && tt %2 == 0)
//            l[i] = (int)((double)dmax*rnd());
        
        if(l[i] >= dmax)
            l[i] = dmax - 1;
        if(l[i] < 0)
            l[i] = 0;
    }
    
    for(k=0;k<ramda[net->layer];k++){
        dis[0] = 0.0;
        
        v[l[k]][0] = P[l[k]][0];
        v[l[k]][1] = P[l[k]][1];
        v[l[k]][2] = P[l[k]][2];
        for(i=0;i<3;i++){
            dis[0] += net->weight[i]*(net->node[0][i] - v[l[k]][i])*(net->node[0][i] - v[l[k]][i]);
        }
        
//        if((net->node[0][1] < -50 && net->node[0][0] > 50))
//            dis[0] = dis[0]*15;
        mindis = dis[0];
        s1 = 0;
        
        for (i=1;i<net->node_n;i++){
            dis[i] = 0;
            
            for(j=0;j<3;j++)
                dis[i] += net->weight[j]*(net->node[i][j] - v[l[k]][j])*(net->node[i][j] - v[l[k]][j]);
            
//            if((net->node[i][1] < -50 && net->node[i][0] > 50))
//                dis[i] = dis[i]*15;
            if(dis[i] < mindis){
                mindis = dis[i];
                s1 = i;
            }
        }
        
        
        
        if(s1 != 0){
            mindis2 = dis[0];
            s2 = 0;
        }else{
            mindis2 = dis[1];
            s2 = 1;
        }
        
        for (i=0;i<net->node_n;i++){
            if(dis[i] < mindis2 && i != s1){
                mindis2 = dis[i];
                s2 = i;
            }
        }
        
//        if(rangeArea(net->node[s1][0],net->node[s1][1],net->node[s1][2],0,30,170,250,250,50) == 0){
//            if(rnd() > net->strength[s1]){
//                continue;
//            }
//        }
        
//        if(rangeArea(net->node[s1][0],net->node[s1][1],net->node[s1][2],0,30,170,250,250,50)){
            dis[s1] = dis[s1];
            mindis2 *= pow((4*net->strength[s1]),4);
            mindis *= pow((4*net->strength[s1]),4);
//        }
        
        net->gng_err[s1] += mindis;
        net->gng_u[s1] += mindis2-mindis;
        
        net->wct[s1]++;
        net->wct[s2]++;
        
        //        gng_learn(net, s1, v[l[k]], 0.08, 0.008, 0, 6);
        gng_learn_local_fcm(net, s1, s2, v[l[k]], dis);
        net->node_n = calc_age(s1, s2, net);
        discount_err_gng(net);
        if(net->node_n > 10 && flag == 1){
            node_delete_u(net);
        }
        
    }
    
    last_node = net->node_n;
    if (last_node != net->node_n) {
        printf("test");
    }
    
    if(net->node_n < GNGN-1)
        node_add_gng(net, flag);
    if(net->node_n < GNGN)
        node_add_gng2(net, flag);
//    if(net->node_n < GNGN)
//        node_add_gng2(net, flag);
    
    //    int dlist[GNGN];
    //    int dct = 0;
    //
    //    for (i=0;i<net->node_n-1;i++){
    //        if(net->wct[i] == 0){
    //            dlist[dct] = i;
    //            dct++;
    //        }else{
    //            net->wct[i] = 0;
    //        }
    //    }
    //
    //    if(dct != 0){
    //        for(i=0;i<dct;i++){
    //            node_delete(net, dlist[i]-i);
    //        }
    //    }
    
    
    //    if(net->node_n > 10){
    //        double adis = 0.0;
    //
    //        for(i=0;i<net->node_n;i++){
    //            double tmpdis[16] = {0.0};
    //            int tmpct = 0;
    //            int tmplist[16] = {0};
    //            adis = 0.0;
    //            for(j=0;j<net->node_n;j++){
    //                tmpdis[tmpct] = 0.0;
    //
    //                if(net->edge[i][j] == 1){
    //                    for(k=0;k<3;k++) tmpdis[tmpct] += (net->node[i][k] - net->node[j][k])*(net->node[i][k] - net->node[j][k]);
    //                    tmpdis[tmpct] = sqrt(tmpdis[tmpct]);
    //                    adis += tmpdis[tmpct];
    //                    tmplist[tmpct] = j;
    //                    tmpct++;
    //                }
    //            }
    //
    //            if(tmpct > 2){
    //                adis /= (double)tmpct;
    //                for(j=0;j<tmpct;j++){
    //                    k = tmplist[j];
    //                    if(adis/tmpdis[j] < 0.8){
    //                        net->edge[i][k] = 0;
    //                        net->edge[k][i] = 0;
    //                    }
    //                }
    //            }
    //        }
    //    }
    //
    //
//    for(k=0;k<ramda[net->layer];k++){
//        dis[0] = 0.0;
//        for(i=0;i<3;i++){
//            dis[0] += net->weight[i]*(net->node[0][i] - v[l[k]][i])*(net->node[0][i] - v[l[k]][i]);
//        }
//
//        mindis = dis[0];
//        s1 = 0;
//
//        for (i=1;i<net->node_n;i++){
//            dis[i] = 0;
//
//            for(j=0;j<3;j++)
//                dis[i] += net->weight[j]*(net->node[i][j] - v[l[k]][j])*(net->node[i][j] - v[l[k]][j]);
//            if(dis[i] < mindis){
//                mindis = dis[i];
//                s1 = i;
//            }
//        }
//        
//        gng_learn(net, s1, v[l[k]], 0.01, 0.0, 0, 3);
//    }
    
    free2d_double(v);
    return net->node_n;
}

void gng_learn_local(struct gng *net, int s1, int s2, double *v, double *h, double **h2, double *dis)
{
    int i,j;
    double e1;
    int cct = 0;
    int *clist = (int *)malloc(sizeof(int)*net->node_n);
    double *mu = (double *)malloc(sizeof(double)*net->node_n);
    const int m = 2;
    const double ETA = 1.0;
    
    if(dis[s1] != 0){
        clist[cct] = s1;
        cct++;
    }else{
        e1 = ETA;
        h[s1] += e1;
        
        for(i=0;i<DIM;i++){
            h2[s1][i] += e1*(v[i]-net->node[s1][i]);
        }
        
        free(mu);
        free(clist);
        return;
    }
    
    if(dis[s2] != 0){
        clist[cct] = s2;
        cct++;
    }
    
    for(i=0;i<net->node_n;i++){
        if(net->edge[s1][i] == 1 && s1 != i && s2 != i){
            if(dis[i] != 0){
                clist[cct] = i;
                cct++;
            }
        }
    }
    
    for(i=0;i<cct;i++){
        mu[i] = 0.0;
        for(j=0;j<cct;j++){
            mu[i] += pow(dis[clist[i]]/dis[clist[j]], 1.0/((double)m-1.0));
        }
        mu[i] = 1.0/mu[i];
        e1 = ETA;
        for(j=0;j<m;j++)
            e1 *= mu[i];
        h[clist[i]] += e1;
        for(j=0;j<DIM;j++)
            h2[clist[i]][j] += e1*(v[j]-net->node[clist[i]][j]);
    }
    
    free(mu);
    free(clist);
}

int calc_age_local(int s1, int s2, struct gng *net, int **edge, int **age)
{
    int i;
    
    edge[s1][s2]++;
    edge[s2][s1]++;
    for(i=0;i<net->node_n;i++){
        if(i != s1 && i != s2){
            if(net->edge[s1][i] == 1){
                age[s1][i]++;
                age[i][s1]++;
            }
        }
    }
    
    return net->node_n;
}

int node_delete_batch(struct gng *net, int **edge, int **age, int *ed, int i)
{
    int j,k,l;
    for(j=i;j<net->node_n;j++){
        ed[j] = ed[j+1];
        net->gng_err[j] = net->gng_err[j+1];
        net->gng_u[j] = net->gng_u[j+1];
        net->edge_dimension[j] = net->edge_dimension[j+1];
        net->wct[j] = net->wct[j+1];
        for(l=0;l<DIM;l++){
            net->node[j][l] = net->node[j+1][l];
            net->delta[j][l] = net->delta[j+1][l];
        }
        
        for(k=0;k<net->node_n;k++){
            net->age[j][k] = net->age[j+1][k];
            net->age[k][j] = net->age[k][j+1];
            net->edge[j][k] = net->edge[j+1][k];
            net->edge[k][j] = net->edge[k][j+1];
            age[j][k] = age[j+1][k];
            age[k][j] = age[k][j+1];
            edge[j][k] = edge[j+1][k];
            edge[k][j] = edge[k][j+1];
            
        }
    }
    
    net->node_n--;
    return net->node_n;
}

void calc_age_batch(struct gng *net, int **edge, int **age)
{
    int i,j;
    int *ed = (int *)malloc(sizeof(int)*net->node_n);
    double a_ave = 0.0;
    double r = 1.0;
    double p;
    
    for(i=0;i<net->node_n;i++) ed[i]=net->edge_dimension[i];
    
    for(i=0;i<net->node_n;i++){
        net->edge_dimension[i] = 0;
        for(j=0;j<net->node_n;j++){
            if(i == j) continue;
            
            if(edge[i][j] > 0){
                net->edge[i][j] = 1;
                net->age[i][j] = 0;
            }else{
                net->edge[i][j] = 0;
                net->age[i][j] = 0;
            }
            
            if(net->edge[i][j] == 1){
                net->edge_dimension[i]++;
            }
        }
        if(net->edge_dimension[i] == 0){
            net->node_n = node_delete_batch(net, edge, age, ed, i);
        }
    }
    
    free(ed);
}

void discount_err_blgng(struct gng *net, double rate)
{
    int i;
    for(i=0;i<net->node_n;i++){
        net->gng_err[i] -= rate*net->gng_err[i];
        if(net->gng_err[i] < 0)
            net->gng_err[i] = 0.0;
        
        net->gng_u[i] -= rate*net->gng_u[i];
        if(net->gng_u[i] < 0)
            net->gng_u[i] = 0.0;
    }
}
/*
 int gng_batch_main_s(struct gng *net, Pointcloud &ptc, Pointcloud &cptc, int tt)
 {
 
 //
 const double MOMENTUM = 1.0;
 static int ct;
 //
 
 int i,j,k,k1;
 int s1,s2;
 double mindis, mindis2, *dis;
 int ramda;
 static int ramda2 = 0;
 int t;
 double *h = (double *)malloc(sizeof(double)*net->node_n);
 double **h2;
 int **edge;
 int **age;
 static int initflag = 1;
 double **v;
 h2 = malloc2d_double(net->node_n, DIM);
 edge = malloc2d_int(net->node_n, net->node_n);
 age = malloc2d_int(net->node_n, net->node_n);
 dis = (double *)malloc(sizeof(double)*net->node_n);
 
 int dmax = (int)ptc.size();
 ramda = dmax;
 v = malloc2d_double(dmax, DIM);
 
 for(i=0;i<net->node_n;i++){
 h[i] = 0.0;
 for(j=0;j<DIM;j++) h2[i][j] = 0.0;
 for(j=0;j<net->node_n;j++){
 edge[i][j] = 0;
 age[i][j] = 0;
 }
 }
 
 for(k1=0;k1<ramda;k1++){
 //k = index[(k1+ramda2)%ND];
 k=k1;
 v[k][0] = ptc.getPoint(k).x();
 v[k][1] = ptc.getPoint(k).y();
 v[k][2] = ptc.getPoint(k).z();
 v[k][3] = cptc.getPoint(k).x();
 v[k][4] = cptc.getPoint(k).y();
 v[k][5] = cptc.getPoint(k).z();
 
 //第1勝者ノードの決定
 mindis = 0;
 for(i=0;i<DIM;i++)
 mindis += net->weight[i]*(net->node[0][i] - v[k][i])*(net->node[0][i] - v[k][i]);
 dis[0] = mindis;
 s1 = 0;
 
 for (i=1;i<net->node_n;i++){
 dis[i] = 0;
 for(j=0;j<DIM;j++)
 dis[i] += net->weight[i]*(net->node[i][j] - v[k][j])*(net->node[i][j] - v[k][j]);
 if(dis[i] < mindis){
 mindis = dis[i];
 s1 = i;
 }
 }
 
 net->wct[s1]++;
 
 //第2勝者ノードの決定
 if(s1 != 0){
 mindis2 = dis[0];
 s2 = 0;
 }else{
 mindis2 = dis[1];
 s2 = 1;
 }
 
 for (i=0;i<net->node_n;i++){
 if(dis[i] < mindis2 && i != s1){
 mindis2 = dis[i];
 s2 = i;
 }
 }
 
 //積算誤差の計算
 net->gng_err[s1] += mindis;
 
 //
 net->gng_u[s1] += mindis2 - mindis;
 
 //ノードの更新
 gng_learn_local(net, s1, s2, v[k], h, h2, dis);
 
 //エッジの年齢の更新
 calc_age_local(s1, s2, net, edge, age);
 
 }
 
 //ノードの更新
 double serr = 0.0;
 double sdis;
 if(initflag == 0){
 for (i=0;i<net->node_n;i++)
 if(h[i] != 0){
 for(j=0;j<DIM;j++){
 net->delta[i][j] = MOMENTUM*h2[i][j]/h[i] + (1.0-MOMENTUM)*net->delta[i][j];
 net->node[i][j] += net->delta[i][j];
 sdis += net->delta[i][j]*net->delta[i][j];
 serr += sqrt(sdis);
 }
 }else{
 for(j=0;j<DIM;j++){
 net->delta[i][j] = (1.0-MOMENTUM)*net->delta[i][j];
 net->node[i][j] += net->delta[i][j];
 sdis += net->delta[i][j]*net->delta[i][j];
 serr += sqrt(sdis);
 }
 }
 }else{
 for (i=0;i<net->node_n;i++)
 if(h[i] != 0){
 for(j=0;j<DIM;j++){
 net->delta[i][j] = h2[i][j]/h[i];
 net->node[i][j] += net->delta[i][j];
 sdis += net->delta[i][j]*net->delta[i][j];
 serr += sqrt(sdis);
 }
 }
 }
 
 //    serr /= (double)net->node_n;
 //    printf("S:%lf\n",serr);
 
 calc_age_batch(net, edge, age);
 
 if(initflag == 0){
 if(net->node_n < GNGNB)
 node_add_gng(net, 0);
 //        else
 //            node_delete_gng(net);
 }
 
 //積算誤差の計算
 discount_err_blgng(net, (double)ramda);
 
 if(initflag == 1) initflag = 0;
 
 ramda2 += ramda;
 ct++;
 
 free(h);
 free2d_double(h2);
 free2d_double(v);
 free2d_int(edge);
 free2d_int(age);
 free(dis);
 
 return t;
 }
 
 */

