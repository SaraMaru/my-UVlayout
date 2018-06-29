#include "my_header.h"
using namespace std;

#define VERY_BIG_NUMBER 999999
float margin = 0.03;

void get_bounding_box_for_UV(const UV_list UV, int len, box& b) {
    float max_u,min_u,max_v,min_v;
    max_u = max_v = -VERY_BIG_NUMBER;
    min_u = min_v = VERY_BIG_NUMBER;

    for(int i=0; i<len; i++) {
        if(UV[i].x()>max_u) max_u = UV[i].x();
        if(UV[i].x()<min_u) min_u = UV[i].x();
        if(UV[i].y()>max_v) max_v = UV[i].y();
        if(UV[i].y()<min_v) min_v = UV[i].y();
    }

    b.min_u = min_u;
    b.min_v = min_v;
    b.width = max_u - min_u;
    b.height = max_v - min_v;
}

bool collide(box &a, box &b) {
    if(a.min_u>=b.min_u+b.width || a.min_u+a.width<=b.min_u || a.min_v>=b.min_v+b.height || a.min_v+a.height<=b.min_v)
        return false;
    else
        return true;
}

void fit_in_a_box(int len, const box &old_box, const UV_list &old_UV, const box &new_box, UV_list &new_UV) {
    float new_min_u = new_box.min_u + margin*new_box.width;
    float old_min_u = old_box.min_u;
    float new_min_v = new_box.min_v + margin*new_box.height;
    float old_min_v = old_box.min_v;
    float height_ratio = (1-2*margin)*new_box.height / old_box.height;
    float width_ratio = (1-2*margin)*new_box.width / old_box.width;
    new_UV = new Eigen::Vector2d[len];
    for(int i=0; i<len; i++) {
        new_UV[i].x() = (old_UV[i].x() - old_min_u) * width_ratio + new_min_u;
        new_UV[i].y() = (old_UV[i].y() - old_min_v) * height_ratio + new_min_v;
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
        get_bounding_box_for_UV(all_UV[c],cl[c].vertices.size(),b);
        old_bl.push_back(b);
        new_bl.push_back(b);
        cout<<"box "<<c<<": "<<b.min_u<<"~"<<b.min_u+b.width<<" "<<b.min_v<<"~"<<b.min_v+b.height<<endl;
    }
    cout<<"get_bounding_box_for_UV() done"<<endl;

    for(int c=0; c<chart_num; c++) { //insert chart_num times
        float current_max_len = VERY_BIG_NUMBER;
        int next_box = -1;
        float next_min_u,next_min_v;
        vector<float> u_list;
        vector<float> v_list;
        u_list.push_back(0);
        v_list.push_back(total_height);
        u_list.push_back(total_width);
        v_list.push_back(0);
        for(int other=0; other<chart_num; other++) {
            if(!is_in[other])
                continue;
            u_list.push_back(new_bl[other].min_u);
            v_list.push_back(new_bl[other].min_v + new_bl[other].height); //the left-top vertex
            u_list.push_back(new_bl[other].min_u + new_bl[other].width);
            v_list.push_back(new_bl[other].min_v); //the right-bottom vertex
        }

        for(int c=0; c<chart_num; c++) {
            if(is_in[c])
                continue;
            float space_min_u, space_min_v;
            float min_max_len = VERY_BIG_NUMBER;
            /*if(u_list.size()==0) {
                space_min_u = space_min_v = 0;
                min_max_len = old_bl[c].height>old_bl[c].width? old_bl[c].height : old_bl[c].width;
            }*/
            box new_box;
            new_box.width = old_bl[c].width;  new_box.height = old_bl[c].height;
            for(int i=0; i<u_list.size(); i++) {
                new_box.min_u = u_list[i];  new_box.min_v = v_list[i];
                //cout<<" "<<c<<"__"<<new_box.min_u<<"__"<<new_box.min_v;
                bool can_insert = true;
                for(int other=0; other<chart_num; other++) {
                    if(is_in[other] && collide(new_box,new_bl[other])) {
                        can_insert = false;
                        break;
                    }
                }
                if(!can_insert)
                    continue;
                //cout<<"<"<<can_insert<<">";
                float new_width = new_box.min_u + new_box.width;
                new_width = new_width>total_width? new_width : total_width;
                float new_height = new_box.min_v + new_box.height;
                new_height = new_height>total_height? new_height : total_height;
                float new_max_len = new_width>new_height? new_width : new_height;
                //cout<<"new"<<new_max_len<<"-min"<<min_max_len<<" ";
                if(new_max_len>=min_max_len)
                    continue;
                min_max_len = new_max_len;
                space_min_u = new_box.min_u; space_min_v = new_box.min_v;
            }
            
            if(min_max_len<current_max_len) {
                current_max_len = min_max_len;
                next_box = c;
                next_min_u = space_min_u;  next_min_v = space_min_v;
            }
        }

        is_in[next_box] = true;
        new_bl[next_box].min_u = next_min_u;  new_bl[next_box].min_v = next_min_v;
        float tmp;
        tmp = next_min_u + old_bl[next_box].width;
        total_width = tmp>total_width? tmp : total_width;
        tmp = next_min_v + old_bl[next_box].height;
        total_height = tmp>total_height? tmp : total_height;
        max_len = current_max_len;
    }

    cout<<"max_len: "<<max_len<<endl;
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
        fit_in_a_box(cl[c].vertices.size(),old_bl[c],all_UV[c],new_bl[c],UV);
        result.push_back(UV);
    }
    cout<<"rectangle_pack() done"<<endl;
}
