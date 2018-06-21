#include "my_header.h"
using namespace std;

#define VERY_BIG_NUMBER 999999

void get_bounding_box_for_UV(const UV_list &UV, box& b) {
    float max_u,min_u,max_v,min_v;
    max_u = max_v = -VERY_BIG_NUMBER;
    min_u = min_v = VERY_BIG_NUMBER;

    for(int i=0; i<old_box.size(); i++) {
        if(UV[i].x>max_u) max_u = UV[i].x;
        if(UV[i].x<min_u) min_u = UV[i].x;
        if(UV[i].y>max_v) max_v = UV[i].y;
        if(UV[i].y<min_v) min_v = UV[i].y;
    }

    b.min_u = min_u;
    b.min_v = min_v;
    b.width = max_u - min_u;
    b.height = max_v - min_v;
}

void fit_in_a_box(const box &old_box, const UV_list &old_UV, const box &new_box, UV_list &new_UV) {
    float new_min_u = new_box.min_u;
    float old_min_u = old_box.min_u;
    float new_min_v = new_box.min_v;
    float old_min_v = old_box.min_v;
    float height_ratio = new_box.height / old_box.height;
    float width_ratio = new_box.width / old_box.width;
    new_UV = new aiVector2D[old_box.size()];
    for(int i=0; i<old_box.size(); i++) {
        new_UV[i].x = (old_UV[i].x - old_min_u) * width_ratio + new_min_u;
        new_UV[i].y = (old_UV[i].y - old_min_v) * height_ratio + new_min_v;
    }
}

/* put all the UVs into a range of 0~1 */
void pack(const chart_list &ch, const scene_UV_list &all_UV, scene_UV_list &result) {
    float total_width,total_height,max_len; //max_len is the bigger one between total_width and total_height
    int chart_num = ch.size();
    bool is_in = new bool[chart_num];
    box_list bl;

    for(int c=0; c<chart_num; c++)
        is_in = false;
    for(int c=0; c<chart_num; c++) {
        box b;
        get_bounding_box_for_UV(all_UV,b);
        bl.push_back(b);
    }

    for(int c=0; c<chart_num; c++) {
        for(int c=0; c<chart_num; c++) {
            if(is_in[box_list])
        }
    }
}
