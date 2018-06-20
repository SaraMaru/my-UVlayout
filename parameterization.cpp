#include "my_header.h"
using namespace Eigen;

#define VERY_BIG_NUMBER 999999

Vector3d trans(const aiVector3D &v) {
    Vector3d vv(v.x,v.y,v.z);
    return vv;
}

double dist(const Vector3d &a, const Vector3d &b) {
    return (a-b).norm();
}

double dist(aiVector3D &a, aiVector3D &b) {
    //return dist(trans(a),trans(b));
    return (a-b).Length();
}

/* each triangle is provided with a local orthonormal basis */
void get_coordinates(const chart &ch, triangle *ts) {
    const aiMesh *mesh = ch.mi.mesh;
    face_normals fn = ch.mi.fn;
    int index = 0;
    for(int f=0; f<mesh->mNumFaces; f++) {
        if(ch.faces.find(f)==ch.faces.end())
            continue;
        const aiFace *face = &mesh->mFaces[f];
        Vector3d v1 = trans(mesh->mVertices[face->mIndices[0]]);
        Vector3d v2 = trans(mesh->mVertices[face->mIndices[1]]);
        Vector3d v3 = trans(mesh->mVertices[face->mIndices[2]]);
        ts[index].x2 = dist(v1,v2);
        Vector3d x_axis = (v2-v1) / ts[index].x2;
        Vector3d v1v3 = v3-v1;
        ts[index].x3 = v1v3.dot(x_axis);
        Vector3d y_axis = trans(fn[index]).cross(x_axis);
        y_axis.normalize();
        ts[index].y3 = v1v3.dot(y_axis);
        std::cout<<"f"<<index<<": ("<<ts[index].x1<<","<<ts[index].y1<<") ("<<ts[index].x2<<","<<ts[index].y2<<") ("<<ts[index].x3<<","<<ts[index].y3<<")"<<std::endl;
        std::cout<<"fn: "<<trans(fn[index]).transpose()<<std::endl;
        index++;
    }
    /*for(int f=0; f<mesh->mNumFaces; f++) {
        if(ch.faces.find(f)==ch.faces.end())
            continue;
        const aiFace *face = &mesh->mFaces[f];
        ts[index].x1 = mesh->mVertices[face->mIndices[0]].x;
        ts[index].y1 = mesh->mVertices[face->mIndices[0]].y;
        ts[index].x2 = mesh->mVertices[face->mIndices[1]].x;
        ts[index].y2 = mesh->mVertices[face->mIndices[1]].y;
        ts[index].x3 = mesh->mVertices[face->mIndices[2]].x;
        ts[index].y3 = mesh->mVertices[face->mIndices[2]].y;
        index++;
    }*/
}

double find_pinned_vertices(const chart &ch, int &a, int &b) {
    const aiMesh *mesh = ch.mi.mesh;
    int max_x,max_y,max_z;
    int min_x,min_y,min_z;
    int max_x_id,max_y_id,max_z_id,min_x_id,min_y_id,min_z_id;
    max_x = max_y = max_z = -VERY_BIG_NUMBER;
    min_x = min_y = min_z = VERY_BIG_NUMBER;
    for(map<int,int>::const_iterator iter=ch.m_2_u.begin(); iter!=ch.m_2_u.end(); iter++) {
        //cout<<iter->first<<"%%"<<iter->second<<endl;
        int i = iter->first;
        aiVector3D v = mesh->mVertices[i];
        //cout<<v.x<<" "<<v.y<<" "<<v.z<<endl;
        if(v.x>max_x) { max_x = v.x; max_x_id = i; }
        if(v.x<min_x) { min_x = v.x; min_x_id = i; }
        if(v.y>max_y) { max_y = v.y; max_y_id = i; }
        if(v.y<min_y) { min_y = v.y; min_y_id = i; }
        if(v.z>max_z) { max_z = v.z; max_z_id = i; }
        if(v.z<min_z) { min_z = v.z; min_z_id = i; }
    }
    double dist_x = dist(mesh->mVertices[min_x_id],mesh->mVertices[max_x_id]);
    double dist_y = dist(mesh->mVertices[min_y_id],mesh->mVertices[max_y_id]);
    double dist_z = dist(mesh->mVertices[min_z_id],mesh->mVertices[max_z_id]);
    double max_dist;
    if(dist_x>dist_z) {
        if(dist_x>dist_y) { //dist_x is the biggest
            a = min_x_id;  b = max_x_id; max_dist = dist_x;
        }
        else { //dist_y is the biggest
            a = min_y_id;  b = max_y_id; max_dist = dist_y;
        }
    }
    else { 
        if(dist_y>dist_z) { //dist_y is the biggest
            a = min_y_id;  b = max_y_id; max_dist = dist_y;
        }
        else { //dist_z is the biggest
            a = min_z_id;  b = max_z_id; max_dist = dist_z;
        }
    }
    if(a>b) {
        int tmp = b;
        b = a;
        a = tmp;
    }
    std::cout<<"NO."<<a<<": "<<trans(mesh->mVertices[a]).transpose()<<endl;
    std::cout<<"NO."<<b<<": "<<trans(mesh->mVertices[b]).transpose()<<endl;
    a = ch.m_2_u.find(a)->second;
    b = ch.m_2_u.find(b)->second;
    std::cout<<"new id: "<<a<<" and "<<b<<endl;
    return max_dist;
}

void parameterize(const chart &ch, UV_list &UV) {
    const aiMesh *mesh = ch.mi.mesh;
    int f_num = ch.faces.size();
    int v_num = ch.m_2_u.size();
    cout<<"f_num: "<<f_num<<" "<<"v_num: "<<v_num<<endl;

    triangle *ts = new triangle[f_num];
    get_coordinates(ch,ts);
    std::cout<<"get_coordinates() done"<<std::endl;

    /*if(f_num==1) {
        UV = new aiVector2D[3];
        return;
    }*/

    MatrixXd Mr = MatrixXd::Zero(f_num,v_num);
    MatrixXd Mi = MatrixXd::Zero(f_num,v_num);

    for(int i=0; i<f_num; i++)
        cout<<ts[i].x1<<" "<<ts[i].y1<<" "<<ts[i].x2<<" "<<ts[i].y2<<" "<<ts[i].x3<<" "<<ts[i].y3<<" "<<endl;

    int f=0;
    for(set<int>::const_iterator it=ch.faces.begin(); it!=ch.faces.end(); it++) {
        const aiFace *face = &mesh->mFaces[*it];
        triangle t = ts[f];
        double sqrt_dt = sqrt( (t.x1*t.y2 - t.y1*t.x2) + (t.x2*t.y3 - t.y2*t.x3) + (t.x3*t.y1 - t.y3*t.x1) );
        int v0 = ch.m_2_u.find(face->mIndices[0])->second;
        int v1 = ch.m_2_u.find(face->mIndices[1])->second;
        int v2 = ch.m_2_u.find(face->mIndices[2])->second;
        cout<<v0<<" "<<v1<<" "<<v2<<endl;
        Mr(f,v0) = (t.x3 - t.x2) / sqrt_dt;
        Mr(f,v1) = (t.x1 - t.x3) / sqrt_dt;
        Mr(f,v2) = (t.x2 - t.x1) / sqrt_dt;
        Mi(f,v0) = (t.y3 - t.y2) / sqrt_dt;
        Mi(f,v1) = (t.y1 - t.y3) / sqrt_dt;
        Mi(f,v2) = (t.y2 - t.y1) / sqrt_dt;
        f++;
    }
    std::cout<<"M done"<<std::endl;
    std::cout<<"Mr: "<<Mr<<endl;
    std::cout<<"Mi: "<<Mi<<endl;

    int id1,id2;
    double max_dist = find_pinned_vertices(ch, id1, id2);
    std::cout<<"find_pinned_vertices() done"<<std::endl;
    std::cout<<"max_dist: "<<max_dist<<endl;

    MatrixXd Mpr(f_num,2);
    Mpr<<Mr.col(id1),Mr.col(id2);
    //std::cout<<"A"<<std::endl;
    MatrixXd Mpi(f_num,2);
    Mpi<<Mi.col(id1),Mi.col(id2);
    //std::cout<<"B"<<std::endl;
    MatrixXd Mfr = MatrixXd::Zero(f_num,v_num-2);
    MatrixXd Mfi = MatrixXd::Zero(f_num,v_num-2);
    //std::cout<<"C"<<id1<<std::endl;
    if(id1>0) {
        Mfr.block(0,0,f_num,id1) = Mr.block(0,0,f_num,id1);
        Mfi.block(0,0,f_num,id1) = Mi.block(0,0,f_num,id1);
    }
    //std::cout<<"D"<<id2<<std::endl;
    if(id2>id1+1) {
        Mfr.block(0,id1,f_num,id2-id1-1) = Mr.block(0,id1+1,f_num,id2-id1-1);
        Mfi.block(0,id1,f_num,id2-id1-1) = Mi.block(0,id1+1,f_num,id2-id1-1);
    }
    //std::cout<<"E"<<std::endl;
    if(id2<f_num-1) {
        Mfr.block(0,id2-1,f_num,v_num-1-id2) = Mr.block(0,id2+1,f_num,v_num-1-id2);
        Mfi.block(0,id2-1,f_num,v_num-1-id2) = Mi.block(0,id2+1,f_num,v_num-1-id2);
    }
    //std::cout<<"F"<<std::endl;

    MatrixXd A(2*f_num,2*(v_num-2));
    A << Mfr, -Mfi,
         Mfi, Mfr;
    MatrixXd b_left(2*f_num,4);
    b_left << Mpr, -Mpi,
              Mpi, Mpr;
    MatrixXd Up(4,1);
    Up << -max_dist/2,max_dist/2,-max_dist/2,max_dist/2;
    MatrixXd b = -b_left*Up; /* b is 2*f_num X 1 */
    std::cout<<"A and b done"<<std::endl;
    std::cout<<"A: "<<A<<endl;
    std::cout<<"b_left: "<<b_left<<endl;
    std::cout<<"b: "<<b<<endl;

    MatrixXd X = (A.transpose()*A).inverse() * (A.transpose()) * b; /* X is 2*(v_num-2) X 1 */

    UV = new aiVector2D[v_num];
    UV[id1] = aiVector2D(Up(0),Up(2));
    UV[id2] = aiVector2D(Up(1),Up(3));
    int UVid = 0;
    for(int index = 0; index<v_num-2; index++) {
        while(UVid==id1 || UVid==id2) {
            std::cout<<UVid<<":("<<UV[UVid].x<<") ("<<UV[UVid].y<<")"<<std::endl;
            UVid++;
        }
        UV[UVid] = aiVector2D(X(index),X(v_num-2+index));
        std::cout<<UVid<<":("<<UV[UVid].x<<") ("<<UV[UVid].y<<")"<<std::endl;
        UVid++;
    }
    std::cout<<id2<<":("<<UV[id2].x<<") ("<<UV[id2].y<<")"<<std::endl;
}

void scene_parameterize (const scene_info &si, chart_list &all_charts, scene_UV_list &SUVL) {
    if(SUVL.size()>0 || all_charts.size()==0)
        return;
    for(chart_list::const_iterator iter=all_charts.begin(); iter!=all_charts.end(); iter++) {
        cout<<"chart "<<iter-all_charts.begin()<<" :"<<endl;
        UV_list UV;
        parameterize(*iter,UV);
        SUVL.push_back(UV);
    }
}