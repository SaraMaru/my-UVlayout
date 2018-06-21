#include "my_header.h"
using namespace std;

#define VERY_BIG_NUMBER 999999
float margin = 0;

void get_bounding_box_for_UV(const UV_list UV, int len, box& b) {
    float max_u,min_u,max_v,min_v;
    max_u = max_v = -VERY_BIG_NUMBER;
    min_u = min_v = VERY_BIG_NUMBER;

    for(int i=0; i<len; i++) {
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

void fit_in_a_box(int len, const box &old_box, const UV_list &old_UV, const box &new_box, UV_list &new_UV) {
    float new_min_u = new_box.min_u + margin*new_box.width;
    float old_min_u = old_box.min_u;
    float new_min_v = new_box.min_v + margin*new_box.height;
    float old_min_v = old_box.min_v;
    float height_ratio = (1-2*margin)*new_box.height / old_box.height;
    float width_ratio = (1-2*margin)*new_box.width / old_box.width;
    new_UV = new aiVector2D[len];
    for(int i=0; i<len; i++) {
        new_UV[i].x = (old_UV[i].x - old_min_u) * width_ratio + new_min_u;
        new_UV[i].y = (old_UV[i].y - old_min_v) * height_ratio + new_min_v;
    }
}

/* put all the UVs into a range of 0~1 */
void rectangle_pack(const chart_list &cl, const scene_UV_list &all_UV, scene_UV_list &result) {
    float total_width, total_height, max_len; //max_len is the bigger one between total_width and total_height
    total_width = total_height = max_len = 0;
    int chart_num = cl.size();
    bool *is_in = new bool[chart_num];
    box_list old_bl;
    box_list new_bl;

    /* initialization */
    for(int c=0; c<chart_num; c++) {
        is_in[c] = false;
        box b;
        get_bounding_box_for_UV(all_UV[c],cl[c].m_2_u.size(),b);
        old_bl.push_back(b);
        new_bl.push_back(b);
        cout<<"box "<<c<<": "<<b.min_u<<"~"<<b.min_u+b.width<<" "<<b.min_v<<"~"<<b.min_v+b.height<<endl;
    }
    cout<<"get_bounding_box_for_UV() done"<<endl;

    for(int c=0; c<chart_num; c++) { //insert chart_num times
        float current_max_len = VERY_BIG_NUMBER;
        int next_box = -1;
        insert_mode mode;
        for(int c=0; c<chart_num; c++) {
            if(!is_in[c]) {
                float new_width = total_width + old_bl[c].width;
                float new_height = total_height + old_bl[c].height;
                float new_max_len = new_width<new_height? new_width : new_height;
                if(new_max_len<current_max_len) {
                    current_max_len = new_max_len;
                    next_box = c;
                    mode = new_width<new_height? INSERT_RIGHT : INSERT_TOP;
                }
            }
        }
        is_in[next_box] = true;
        if(mode==INSERT_RIGHT) {
            new_bl[next_box].min_u = total_width;
            new_bl[next_box].min_v = 0;
            total_width += old_bl[next_box].width;
            if(old_bl[next_box].height>total_height)
                total_height = old_bl[next_box].height;
        }
        else {
            new_bl[next_box].min_u = 0;
            new_bl[next_box].min_v = total_height;
            total_height += old_bl[next_box].height;
            if(old_bl[next_box].width>total_width)
                total_width = old_bl[next_box].width;
        }
        max_len = total_width<total_height? total_width : total_height;
    }

    for(int c=0; c<chart_num; c++) {
        new_bl[c].min_u /= max_len;
        new_bl[c].min_v /= max_len;
        new_bl[c].width /= max_len;
        new_bl[c].height /= max_len;
        cout<<"box "<<c<<": "<< new_bl[c].min_u<<"~"<< new_bl[c].min_u+ new_bl[c].width
            <<" "<< new_bl[c].min_v<<"~"<< new_bl[c].min_v+ new_bl[c].height<<endl;
    }

    for(int c=0; c<chart_num; c++) {
        UV_list UV;
        fit_in_a_box(cl[c].m_2_u.size(),old_bl[c],all_UV[c],new_bl[c],UV);
        result.push_back(UV);
    }
    cout<<"rectangle_pack() done"<<endl;
}
