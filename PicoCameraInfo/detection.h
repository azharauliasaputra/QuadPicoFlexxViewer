//
//  detection.hpp
//  PicoCameraInfo
//
//  Created by Azhar Aulia Saputra on 2019/01/28.
//  Copyright © 2019 Azhar Aulia Saputra. All rights reserved.
//

#ifndef detection_h
#define detection_h

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <iostream>

#include "gng.hpp"
#include "rnd.h"
#include "projection.h"
#include "malloc.h"
#include "picoCamera.h"


int calculateSimilarity(struct gng *net, int cl, int t2);
void checkSimilarityFromNeighbor(struct gng *net, int id);
void gng_plane_clustering(struct gng *net);
void checkSimilarityFromNeighborLabel(struct gng *net, int id);
void gng_suspected_object(struct gng *net);


void searchConnectedRung(int id, struct gng *net, double *n, int n1, int n2, bool &connected);
void updateConnectedRung(int id, struct gng *net, double *n, int n1);
void delete_rung(struct gng *net, int i);
int calc_similirity(struct gng *net, int id);
void rung_update_position(struct gng *net);

double draw_RUNG(struct gng *net, int n1, int color);
int rung_calc_similirity(struct gng *net, double **eval);
void rung_calc_age(struct gng *net);
void generateMultiRung(struct gng *net);
void rung_affordance_detection(struct gng *net);


#endif /* detection_hpp */
