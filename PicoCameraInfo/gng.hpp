/*
 *  gng.h
 *  Claster
 *
 *  Created by Naoyuki Kubota on 12/05/18.
 *  Copyright 2012 首都大学東京. All rights reserved.
 *
 */


#define GNGN 1000
#define RN 1000
#define CN 1000
#define DIM 3

#define GNGNB 50

#define INIT_RUNG_AGE 20
#define THRES_RUNG_AGE 50
#define MAX_RUNG_AGE 150

//#define GROUND_LIMIT (-212.0)// with cart
#define GROUND_LIMIT (-144.0)
#define LEFT_LIMIT -200
#define RIGHT_LIMIT 200
#define LENGTH_LIMIT 400

struct gng {
	double node[GNGN][DIM];     //ノード位置
	
    int edge[GNGN][GNGN];       //エッジ(1:あり，0なし)
    int edgeTriangle[GNGN][GNGN];       //エッジ(1:あり，0なし)
    double normTriangle[GNGN][4];       //エッジ(1:あり，0なし)
    int age[GNGN][GNGN];        //エッジの年齢
    int rung[10][6];        //エッジの年齢
	int node_n;                 //ノード数
    int triangle[GNGN*10][7];
    double triangle_area[GNGN];
//    int square[GNGN][4];
    int intentional_n;
    int intNode_n;
    int triangle_n;
    int square_n;
	int cluster[CN][GNGN];
	int cluster_num[CN];
	int cluster_ct;
    double weight[DIM];         //学習の重み
	double gng_err[GNGN];       //ノードの積算誤差
    double gng_u[GNGN];         //utility valiables
    double strength[GNGN];         //utility valiables
//    struct gng *child;          //子ノード
//    struct gng *parent;         //親ノード
	int layer;                  //層の数
    
    double delta[GNGN][DIM];	//更新量記憶用
    int edge_dimension[GNGN];	//エッジの次数
    
    
    double rung_node[RN][2][DIM];
    double rung_eval[RN][3];
    int rung_node_list[RN][GNGN];
    int rung_nodeCon[RN][GNGN];
    int rung_ct[RN];
    int rung_age[RN];
    int rung_num;
    
    int rung_affordance_ct[RN];
    double rung_affordance_point[RN][RN][DIM];
    double rung_affordance_posi[RN][RN];
    //追加（2016/2/16）
    double prenode[GNGN][DIM];  //t-1のノード位置
    double dir[GNGN][3];        //移動方向
    double movedis[GNGN];       //移動距離の2乗
    int maxMDnode;           //最大の移動距離を持つノード番号
    int maxCVnode;           //最大の曲率を持つノード番号
    
    double cog[3];          //注視領域
    double sigma;           //注視領域の分散値
    
    double K;               //Utilityの係数
    
    double safedeg[GNGN];       //移動距離の2乗
    int label[CN];
    double label_node[CN][DIM];
    double label_norm[CN][DIM];
    int label_list[CN][GNGN];
    int label_ct[CN];
    double label_area[CN];
    int label_edge[CN][CN];
    int label_num;
    
    int sus_node[GNGN];
    int sus_node_num;
    
    int obs_count;
    int sus_object_num;
    int sus_object_ct[CN];
    int sus_object_list[CN][GNGN];
    double sus_object_size[CN][DIM][2];
    
    int wct[GNGN];
    
    double maxE;
    int maxE_n;
    
    //追加
    double udrate;
    
};

extern double time_strength;
extern struct gng *ocnet;
extern struct gng *intnet;
struct gng *init_gng();
int gng_main(struct gng *net, double v[][DIM],int tt, int dmax);
void generateMultiRung(struct gng *net);
void gng_suspected_object(struct gng *net);

void gng_classification(struct gng *net);
void gng_plane_clustering(struct gng *net);
void gng_triangulation(struct gng *net);
struct gng *hgng_main(struct gng *net, double v[][DIM],int tt, int dmax);
void connect_colordata(struct gng *net, int color[][3], int color_cluster[][3], int dmax);
void connect_distancedata(struct gng *net, double v[][DIM], int distance_cluster[], int dmax, int dim_s, int dim_f);

int gng_main_oct(struct gng *net, double P[4 * 171 * 224][DIM], int dmax, int tt, int flag);
void gng_triangle_search(struct gng *net);
void intentionalRawData(struct gng *net, double P[153216][DIM]);
int rangeArea(double x, double y, double z, double x_pos, double y_pos, double z_pos, double x_size, double y_size, double z_size);
int rangeArea2(double x, double y, double z, double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);

//追加
int search_maxMD(struct gng *net);
int search_maxCurvature(struct gng *net);
int gng_main_c(struct gng *net, double v[][DIM],int tt, int dmax);
int search_maxSafedeg(struct gng *net);
void calc_safe_area(struct gng *net);
void calc_surface_label(struct gng *net);
int label_to_data(struct gng *net, double v[]);
void label_sort(struct gng *net);
void gng_calc_strengh(struct gng *net, int s1);


//int gng_main_oct_test(struct gng *net, Pointcloud &ptc, Pointcloud &cptc,int tt, int flag);
//
//int gng_batch_main_s(struct gng *net, Pointcloud &ptc, Pointcloud &cptc, int tt);
