#include "my_header.h"
#include "Eigen/Dense"
using namespace Eigen;

#define VERY_BIG_NUMBER 999999

Vector3d trans(aiVector3D &v) {
    Vector3d vv(v.x,v.y,v.z);
    return vv;
}

double dist(Vector3d &a, Vector3d &b) {
    return (a-b).norm();
}

double dist(aiVector3D &a, aiVector3D &b) {
    return (trans(a)-trans(b)).norm();
}

/* each triangle is provided with a local orthonormal basis */
void get_coordinates(const mesh_info &mi, triangle *ts) {
    const aiMesh *mesh = mi.mesh;
    face_normals fn = mi.fn;
    for(int f=0; f<mesh->mNumFaces; f++) {
        const aiFace *face = &mesh->mFaces[f];
        Vector3d v1 = trans(mesh->mVertices[face->mIndices[0]]);
        Vector3d v2 = trans(mesh->mVertices[face->mIndices[1]]);
        Vector3d v3 = trans(mesh->mVertices[face->mIndices[2]]);
        ts[f].x2 = dist(v1,v2);
        Vector3d x_axis = (v2-v1) / ts[f].x2;
        Vector3d v1v3 = v3-v1;
        ts[f].x3 = v1v3.dot(x_axis);
        Vector3d y_axis = trans(fn[f]).cross(x_axis);
        y_axis.normalize();
        ts[f].y3 = v1v3.dot(y_axis);
        std::cout<<"f"<<f<<": "<<ts[f].x1<<","<<ts[f].y1<<" "<<ts[f].x2<<","<<ts[f].y2<<" "<<ts[f].x3<<","<<ts[f].y3<<std::endl;
        std::cout<<"fn: "<<trans(fn[f]).transpose()<<std::endl;
    }
    /*for(int f=0; f<mesh->mNumFaces; f++) {
        const aiFace *face = &mesh->mFaces[f];
        ts[f].x1 = mesh->mVertices[face->mIndices[0]].x;
        ts[f].y1 = mesh->mVertices[face->mIndices[0]].y;
        ts[f].x2 = mesh->mVertices[face->mIndices[1]].x;
        ts[f].y2 = mesh->mVertices[face->mIndices[1]].y;
        ts[f].x3 = mesh->mVertices[face->mIndices[2]].x;
        ts[f].y3 = mesh->mVertices[face->mIndices[2]].y;
    }*/
}

void find_pinned_vertices(const aiMesh *mesh, int &a, int &b) {
    int max_x,max_y,max_z;
    int min_x,min_y,min_z;
    int max_x_id,max_y_id,max_z_id,min_x_id,min_y_id,min_z_id;
    max_x = max_y = max_z = -VERY_BIG_NUMBER;
    min_x = min_y = min_z = VERY_BIG_NUMBER;
    for(int i=0; i<mesh->mNumVertices; i++) {
        aiVector3D v = mesh->mVertices[i];
        if(v.x>max_x) { max_x = v.x; max_x_id = i; }
        else if(v.x<min_x) { min_x = v.x; min_x_id = i; }
        if(v.y>max_y) { max_y = v.y; max_y_id = i; }
        else if(v.y<min_y) { min_y = v.y; min_y_id = i; }
        if(v.z>max_z) { max_z = v.z; max_z_id = i; }
        else if(v.z<min_z) { min_z = v.z; min_z_id = i; }
    }
    double dist_x = dist(mesh->mVertices[min_x_id],mesh->mVertices[max_x_id]);
    double dist_y = dist(mesh->mVertices[min_y_id],mesh->mVertices[max_y_id]);
    double dist_z = dist(mesh->mVertices[min_z_id],mesh->mVertices[max_z_id]);
    if(dist_x>dist_z) {
        if(dist_x>dist_y) { //dist_x is the biggest
            a = min_x_id;  b = max_x_id;
        }
        else { //dist_y is the biggest
            a = min_y_id;  b = max_y_id;
        }
    }
    else { 
        if(dist_y>dist_z) { //dist_y is the biggest
            a = min_y_id;  b = max_y_id;
        }
        else { //dist_z is the biggest
            a = min_z_id;  b = max_z_id;
        }
    }
    if(a>b) {
        int tmp = b;
        b = a;
        a = tmp;
    }
}

void parameterize(const mesh_info &mi) {
    const aiMesh *mesh = mi.mesh;
    int v_num = mesh->mNumVertices;
    int f_num = mesh->mNumFaces;

    triangle *ts = new triangle[f_num];
    get_coordinates(mi,ts);
    std::cout<<"get_coordinates() done"<<std::endl;

    MatrixXd Mr = MatrixXd::Zero(f_num,v_num);
    MatrixXd Mi = MatrixXd::Zero(f_num,v_num);
    for(int f=0; f<f_num; f++) {
        const aiFace *face = &mesh->mFaces[f];
        triangle t = ts[f];
        double sqrt_dt = sqrt( (t.x1*t.y2 - t.y1*t.x2) + (t.x2*t.y3 - t.y2*t.x3) + (t.x3*t.y1 - t.y3*t.x1) );
        Mr(f,face->mIndices[0]) = (t.x3 - t.x2) / sqrt_dt;
        Mr(f,face->mIndices[1]) = (t.x1 - t.x3) / sqrt_dt;
        Mr(f,face->mIndices[2]) = (t.x2 - t.x1) / sqrt_dt;
        Mi(f,face->mIndices[0]) = (t.y3 - t.y2) / sqrt_dt;
        Mi(f,face->mIndices[1]) = (t.y1 - t.y3) / sqrt_dt;
        Mi(f,face->mIndices[2]) = (t.y2 - t.y1) / sqrt_dt;
    }
    std::cout<<"M done"<<std::endl;
    std::cout<<Mr<<endl;
    std::cout<<Mi<<endl;

    int id1,id2;
    find_pinned_vertices(mesh, id1, id2);
    std::cout<<"find_pinned_vertices() done"<<std::endl;
    std::cout<<trans(mesh->mVertices[id1])<<endl;
    std::cout<<trans(mesh->mVertices[id2])<<endl;

    MatrixXd Mpr(f_num,2);
    Mpr<<Mr.col(id1),Mr.col(id2);
    MatrixXd Mpi(f_num,2);
    Mpi<<Mi.col(id1),Mi.col(id2);
    MatrixXd Mfr = MatrixXd::Zero(f_num,v_num-2);
    MatrixXd Mfi = MatrixXd::Zero(f_num,v_num-2);
    if(id1>0) {
        Mfr.block(0,0,f_num,id1) = Mr.block(0,0,f_num,id1);
        Mfi.block(0,0,f_num,id1) = Mi.block(0,0,f_num,id1);
    }
    if(id2>id1+1) {
        Mfr.block(0,id1,f_num,id2-id1-1) = Mr.block(0,id1+1,f_num,id2-id1-1);
        Mfi.block(0,id1,f_num,id2-id1-1) = Mi.block(0,id1+1,f_num,id2-id1-1);
    }
    if(id2<f_num-1) {
        Mfr.block(0,id2-1,f_num,v_num-1-id2) = Mr.block(0,id2+1,f_num,v_num-1-id2);
        Mfi.block(0,id2-1,f_num,v_num-1-id2) = Mi.block(0,id2+1,f_num,v_num-1-id2);
    }

    MatrixXd A(2*f_num,2*(v_num-2));
    A << Mfr, -Mfi,
         Mfi, Mfr;
    MatrixXd b_left(2*f_num,4);
    b_left << Mpr, -Mpi,
              Mpi, Mpr;
    MatrixXd Up(4,1);
    Up << -5,5,-5,5;
    MatrixXd b = -b_left*Up; /* b is 2*f_num X 1 */
    std::cout<<"A and b done"<<std::endl;
    std::cout<<A<<endl;
    std::cout<<b_left<<endl;
    std::cout<<b<<endl;

    MatrixXd X = (A.transpose()*A).inverse() * (A.transpose()) * b; /* X is 2*(v_num-2) X 1 */
    std::cout<<X<<endl;
}

void scene_parameterize (const scene_info &si) {
    for(int m=0; m<si.sc->mNumMeshes; m++) {
        mesh_info mi = mesh_info(si.sc->mMeshes[m], si.s_el[m], si.s_efm[m], si.s_refm[m], si.s_vem[m], si.s_fn[m]);
        parameterize(mi);
    }
}