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
        for(int i=0; i<3; i++) {
            Vector3d v1 = trans(mesh->mVertices[face->mIndices[i]]);
            Vector3d v2 = trans(mesh->mVertices[face->mIndices[(i+1)%3]]);
            Vector3d v3 = trans(mesh->mVertices[face->mIndices[(i+2)%3]]);
            Vector3d x_axis = v2-v1;
            x_axis.normalize();
            Vector3d y_axis = trans(fn[index]).cross(x_axis);
            y_axis.normalize(); //actually not needed
            Vector3d v1v3 = v3-v1;
            float x = v1v3.dot(x_axis);
            float y = v1v3.dot(y_axis);
            ts[index][(i+2)%3] = Vector2d(x,y);
        }
        /*std::cout<<"f"<<index<<": ("<<ts[index][0].transpose()<<") ("<<ts[index][1].transpose()<<") ("
            <<ts[index][2].transpose()<<")"<<std::endl;
        std::cout<<"fn: "<<trans(fn[index]).transpose()<<std::endl;*/
        index++;
    }
}

void old_get_coordinates(const chart &ch, triangle *ts) {
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
        ts[index][0][0] = ts[index][0][1] = ts[index][1][1] = 0;
        ts[index][1][0] = dist(v1,v2);
        Vector3d x_axis = (v2-v1) / ts[index][1][0];
        Vector3d v1v3 = v3-v1;
        ts[index][2][0] = v1v3.dot(x_axis);
        Vector3d y_axis = trans(fn[index]).cross(x_axis);
        y_axis.normalize(); //actually not needed
        ts[index][2][1] = v1v3.dot(y_axis);
        //std::cout<<"f"<<index<<": ("<<ts[index][0][0]<<","<<ts[index][0][1]<<") ("<<ts[index][1][0]<<","<<ts[index][1][1]<<") ("<<ts[index][2][0]<<","<<ts[index][2][1]<<")"<<std::endl;
        //std::cout<<"fn: "<<trans(fn[index]).transpose()<<std::endl;
        index++;
    }
    /*for(int f=0; f<mesh->mNumFaces; f++) {
        if(ch.faces.find(f)==ch.faces.end())
            continue;
        const aiFace *face = &mesh->mFaces[f];
        ts[index][0][0] = mesh->mVertices[face->mIndices[0]][0];
        ts[index][0][1] = mesh->mVertices[face->mIndices[0]][1];
        ts[index][1][0] = mesh->mVertices[face->mIndices[1]][0];
        ts[index][1][1] = mesh->mVertices[face->mIndices[1]][1];
        ts[index][2][0] = mesh->mVertices[face->mIndices[2]][0];
        ts[index][2][1] = mesh->mVertices[face->mIndices[2]][1];
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
    std::cout<<"NO."<<a<<": "<<trans(mesh->mVertices[a]).transpose()<<endl;
    std::cout<<"NO."<<b<<": "<<trans(mesh->mVertices[b]).transpose()<<endl;
    a = ch.m_2_u.find(a)->second;
    b = ch.m_2_u.find(b)->second;
    if(a>b) {
        int tmp = b;
        b = a;
        a = tmp;
    }
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

    int id1,id2;
    double max_dist = find_pinned_vertices(ch, id1, id2);
    std::cout<<"find_pinned_vertices() done"<<std::endl;
    std::cout<<"max_dist: "<<max_dist<<endl;

    typedef Triplet<double> T;
    std::vector<T> A_tripletList, b_left_tripletList;
    int f=0;
    for(set<int>::const_iterator it=ch.faces.begin(); it!=ch.faces.end(); it++) {
        const aiFace *face = &mesh->mFaces[*it];
        triangle t = ts[f];
        double dt = (t[0][0]*t[1][1] - t[0][1]*t[1][0]) + (t[1][0]*t[2][1] - t[1][1]*t[2][0]) + (t[2][0]*t[0][1] - t[2][1]*t[0][0]);
        double co = dt>0? 1/sqrt(dt) : -1/(sqrt(-dt));
        for(int i=0; i<3; i++) {
            int j = (i+1)%3;  int k = (i+2)%3;
            int v0 = ch.m_2_u.find(face->mIndices[i])->second;
            double real,image;
            if(dt>0) {
                real = (t[k][0] - t[j][0]) * co;
                image = (t[k][1] - t[j][1]) * co;
            }
            else {
                real = (t[j][1] - t[k][1]) * co;
                image = (t[k][0] - t[j][0]) * co;
            }
            if(v0==id1 || v0==id2) {
                int col = v0==id1? 0 : 1;
                b_left_tripletList.push_back(T(f,col,real));
                b_left_tripletList.push_back(T(f,2+col,-image));
                b_left_tripletList.push_back(T(f_num+f,col,image));
                b_left_tripletList.push_back(T(f_num+f,2+col,real));
            }
            else {
                if(v0>id2) v0-=2;
                else if(v0>id1) v0-=1;
                A_tripletList.push_back(T(f,v0,real));
                A_tripletList.push_back(T(f,v_num-2+v0,-image));
                A_tripletList.push_back(T(f_num+f,v0,image));
                A_tripletList.push_back(T(f_num+f,v_num-2+v0,real));
            }
        }
        f++;
    }
    /*for(int i=0;i<A_tripletList.size();i++)
        std::cout<<A_tripletList[i].row()<<" "<<A_tripletList[i].col()<<" "<<A_tripletList[i].value()<<std::endl;*/
    SparseMatrix<double> A(2*f_num,2*(v_num-2));
    A.setFromTriplets(A_tripletList.begin(), A_tripletList.end());
    std::cout<<"A done"<<std::endl;
    
    std::vector<T> Up_tripletList;
    Up_tripletList.push_back(T(0,0,-max_dist/2));  Up_tripletList.push_back(T(1,0,max_dist/2));
    Up_tripletList.push_back(T(2,0,-max_dist/2));  Up_tripletList.push_back(T(3,0,max_dist/2));
    SparseMatrix<double> Up(4,1);
    Up.setFromTriplets(Up_tripletList.begin(), Up_tripletList.end());
    std::cout<<"Up done"<<std::endl;
    /*for(int i=0;i<b_left_tripletList.size();i++)
        std::cout<<b_left_tripletList[i].row()<<" "<<b_left_tripletList[i].col()<<" "<<b_left_tripletList[i].value()<<std::endl;*/
    SparseMatrix<double> b_left(2*f_num,4);
    b_left.setFromTriplets(b_left_tripletList.begin(), b_left_tripletList.end());
    std::cout<<"b_left done"<<std::endl;
    SparseMatrix<double> b = -b_left*Up; /* b is 2*f_num X 1 */
    std::cout<<"b done"<<std::endl;
    //std::cout<<"A: "<<A<<endl;
    //std::cout<<"b_left: "<<b_left<<endl;
    //std::cout<<"b: "<<b<<endl;

    /* AX = B */
    LeastSquaresConjugateGradient< SparseMatrix<double> > lscg;
    lscg.setMaxIterations(v_num/5); //default is twice the number of columns of the matrix
    lscg.compute(A);
    std::cout<<"compute(A) done"<<std::endl;
    VectorXd X = lscg.solve(b); /* X is 2*(v_num-2) X 1 */
    std::cout<<"X done"<<std::endl;

    UV = new aiVector2D[v_num];
    UV[id1] = aiVector2D(-max_dist/2,-max_dist/2);
    UV[id2] = aiVector2D(max_dist/2,max_dist/2);
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