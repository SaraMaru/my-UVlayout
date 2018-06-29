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
/* only for eigen_parameterize() */
void get_coordinates(const chart &ch, triangle *ts) {
    const aiMesh *mesh = ch.mi.mesh;
    face_normals fn = ch.mi.fn;
    for(int f=0; f<ch.faces.size(); f++) {
        const chart_face &cf = ch.faces[f];
        for(int i=0; i<3; i++) {
            Vector3d v1 = trans(mesh->mVertices[ ch.vertices[cf.indices[i]] ]);
            Vector3d v2 = trans(mesh->mVertices[ ch.vertices[cf.indices[(i+1)%3]] ]);
            Vector3d v3 = trans(mesh->mVertices[ ch.vertices[cf.indices[(i+2)%3]] ]);
            Vector3d x_axis = v2-v1;
            x_axis.normalize();
            Vector3d v1v3 = v3-v1;
            Vector3d z_axis = x_axis.cross(v1v3);
            z_axis.normalize();
            Vector3d y_axis = z_axis.cross(x_axis);
            double x = v1v3.dot(x_axis);
            double y = v1v3.dot(y_axis);
            ts[f][(i+2)%3] = Vector2d(x,y);
        }
        /*std::cout<<"f"<<f<<": ("<<ts[f][0].transpose()<<") ("<<ts[f][1].transpose()<<") ("
            <<ts[f][2].transpose()<<")"<<std::endl;*/
    }
}

/* each triangle is provided with a local orthonormal basis */
/* can be used for both eigen_parameterize() and opennl_parameterize() */
void old_get_coordinates(const chart &ch, triangle *ts) {
    const aiMesh *mesh = ch.mi.mesh;
    face_normals fn = ch.mi.fn;
    for(int f=0; f<ch.faces.size(); f++) {
        const chart_face &cf = ch.faces[f];
        Vector3d v1 = trans(mesh->mVertices[ ch.vertices[cf.indices[0]] ]);
        Vector3d v2 = trans(mesh->mVertices[ ch.vertices[cf.indices[1]] ]);
        Vector3d v3 = trans(mesh->mVertices[ ch.vertices[cf.indices[2]] ]);
        ts[f][0][0] = ts[f][0][1] = ts[f][1][1] = 0;
        ts[f][1][0] = dist(v1,v2);
        Vector3d x_axis = (v2-v1) / ts[f][1][0];
        Vector3d v1v3 = v3-v1;
        ts[f][2][0] = v1v3.dot(x_axis);
        Vector3d z_axis = x_axis.cross(v1v3);
        z_axis.normalize();
        Vector3d y_axis = z_axis.cross(x_axis);
        y_axis.normalize(); //actually not needed
        ts[f][2][1] = v1v3.dot(y_axis);
        /*std::cout<<"f"<<f<<": ("<<ts[f][0][0]<<","<<ts[f][0][1]<<") ("<<ts[f][1][0]<<","<<ts[f][1][1]
            <<") ("<<ts[f][2][0]<<","<<ts[f][2][1]<<")"<<std::endl;*/
    }
}

double find_pinned_vertices(const chart &ch, int &a, int &b, Vector3d &V1, Vector3d &V2) {
    const aiMesh *mesh = ch.mi.mesh;
    double max_x,max_y,max_z;
    double min_x,min_y,min_z;
    int max_x_id,max_y_id,max_z_id,min_x_id,min_y_id,min_z_id;
    max_x = max_y = max_z = -VERY_BIG_NUMBER;
    min_x = min_y = min_z = VERY_BIG_NUMBER;
    for(int i=0; i<ch.vertices.size(); i++) {
        const aiVector3D &v = mesh->mVertices[ch.vertices[i]];
        //cout<<v.x<<" "<<v.y<<" "<<v.z<<endl;
        if(v.x>max_x) { max_x = v.x; max_x_id = i; }
        if(v.x<min_x) { min_x = v.x; min_x_id = i; }
        if(v.y>max_y) { max_y = v.y; max_y_id = i; }
        if(v.y<min_y) { min_y = v.y; min_y_id = i; }
        if(v.z>max_z) { max_z = v.z; max_z_id = i; }
        if(v.z<min_z) { min_z = v.z; min_z_id = i; }
    }
    double dist_x = max_x - min_x;
    double dist_y = max_y - min_y;
    double dist_z = max_z - min_z;
    //cout<<dist_x<<" "<<dist_y<<" "<<dist_z<<endl;
    double max_dist;
    if(dist_x <= dist_y && dist_x <= dist_z) {
        if(dist_y > dist_z) {
            V1 = Vector3d(0,1,0);
            V2 = Vector3d(0,0,1);
            max_dist = dist_y;
            a = min_y_id;  b = max_y_id;
        } else {
            V1 = Vector3d(0,0,1);
            V2 = Vector3d(0,1,0);
            max_dist = dist_z;
            a = min_z_id;  b = max_z_id;
        }
    } else if(dist_y <= dist_x && dist_y <= dist_z) {
        if(dist_x > dist_z) {
            V1 = Vector3d(1,0,0);
            V2 = Vector3d(0,0,1);
            max_dist = dist_x;
            a = min_x_id;  b = max_x_id;
        } else {
            V1 = Vector3d(0,0,1);
            V2 = Vector3d(1,0,0);
            max_dist = dist_z;
            a = min_z_id;  b = max_z_id;
        }
    } else if(dist_z <= dist_x && dist_z <= dist_y) {
        if(dist_x > dist_y) {
            V1 = Vector3d(1,0,0);
            V2 = Vector3d(0,1,0);
            max_dist = dist_x;
            a = min_x_id;  b = max_x_id;
        } else {
            V1 = Vector3d(0,1,0);
            V2 = Vector3d(1,0,0);
            max_dist = dist_y;
            a = min_y_id;  b = max_y_id;
        }
    }
    std::cout<<"V1: "<<V1.transpose()<<", V2: "<<V2.transpose()<<std::endl;
    std::cout<<"min: NO."<<a<<": "<<trans(mesh->mVertices[a]).transpose()<<endl;
    std::cout<<"max: NO."<<b<<": "<<trans(mesh->mVertices[b]).transpose()<<endl;
    std::cout<<"max_dist: "<<max_dist<<endl;
    if(a>b) {
        int tmp = b;
        b = a;
        a = tmp;
    }
    return max_dist;
}

void setup_conformal_map_relations(triangle &t, NLuint v0, NLuint v1, NLuint v2) {
    Vector2d z0,z1,z2;
    z0 = t[0]; z1 = t[1]; z2 = t[2];
    Vector2d z01 = z1 - z0;
    Vector2d z02 = z2 - z0;
    double a = z01[0]; //x2-x1
    double b = z01[1]; //y2-y1
    double c = z02[0]; //x3-x1
    double d = z02[1]; //y3-y1
    //assert(b == 0.0);

    //Note  : 2*id + 0 --> u
    //        2*id + 1 --> v
    NLuint u0_id = 2*v0    ;
    NLuint v0_id = 2*v0 + 1;
    NLuint u1_id = 2*v1    ;
    NLuint v1_id = 2*v1 + 1;
    NLuint u2_id = 2*v2    ;
    NLuint v2_id = 2*v2 + 1;

    //Real part
    nlBegin(NL_ROW);
    nlCoefficient(u0_id, -a+c) ;
    nlCoefficient(v0_id,  b-d) ;
    nlCoefficient(u1_id,   -c) ;
    nlCoefficient(v1_id,    d) ;
    nlCoefficient(u2_id,    a);
    nlEnd(NL_ROW);

    //Imaginary part
    nlBegin(NL_ROW);
    nlCoefficient(u0_id, -b+d);
    nlCoefficient(v0_id, -a+c);
    nlCoefficient(u1_id,   -d);
    nlCoefficient(v1_id,   -c);
    nlCoefficient(v2_id,    a);
    nlEnd(NL_ROW);

    //cout<<u0_id<<" "<<v0_id<<" "<<u1_id<<" "<<v1_id<<" "<<u2_id<<" "<<v2_id<<endl;
    //cout<<a<<" "<<b<<" "<<c<<" "<<d<<endl;
}

void apply(const chart &ch, triangle *ts, int id1, int id2, Vector3d V1, Vector3d V2, UV_list &UV) {
    const aiMesh *mesh = ch.mi.mesh;
	nlNewContext();
    NLuint nb_vertices = NLuint(ch.vertices.size());

    nlSolverParameteri(NL_NB_VARIABLES, NLint(2*nb_vertices));
    nlSolverParameteri(NL_LEAST_SQUARES, NL_TRUE);
    nlSolverParameteri(NL_MAX_ITERATIONS, NLint(20*nb_vertices)); //5*nb_vertices
	nlSolverParameterd(NL_THRESHOLD, 1e-6); //1e-6
    nlBegin(NL_SYSTEM);
    for(int i=0; i<ch.vertices.size(); i++) {
        const aiVector3D &V = mesh->mVertices[ch.vertices[i]];
        double u = trans(V).dot(V1);
        double v = trans(V).dot(V2);
        //std::cout<<"("<<i<<","<<u<<","<<v<<") ";
        nlSetVariable(2 * i    , u);
        nlSetVariable(2 * i + 1, v);
        if(i==id1 || i==id2) {
            nlLockVariable(2 * i    );
            nlLockVariable(2 * i + 1);
        }
    }
    nlBegin(NL_MATRIX);
    for(int f=0; f<ch.faces.size(); f++) {
        NLuint v0 = ch.faces[f].indices[0];
        NLuint v1 = ch.faces[f].indices[1];
        NLuint v2 = ch.faces[f].indices[2];
        setup_conformal_map_relations(ts[f],v0,v1,v2);
	}
    nlEnd(NL_MATRIX);
    nlEnd(NL_SYSTEM);
    std::cout << "Solving ..." << std::endl;
    nlSolve();

    UV = new Vector2d[nb_vertices];
    for(NLuint i=0; i<nb_vertices; ++i) {
        double u = nlGetVariable(2 * i    );
        double v = nlGetVariable(2 * i + 1);
        UV[i] = Vector2d(u,v);
    }

    NLint iterations;	    
    nlGetIntegerv(NL_USED_ITERATIONS, &iterations);
    std::cout<<"Used iterations: "<<iterations<<std::endl;
    nlDeleteContext(nlGetCurrent());
}

/* opennl_parameterize() is much fastr than eigen_parameterize() */
void opennl_parameterize(const chart &ch, UV_list &UV) {
    const aiMesh *mesh = ch.mi.mesh;
    int f_num = ch.faces.size();
    int v_num = ch.vertices.size();
    cout<<"f_num: "<<f_num<<" "<<"v_num: "<<v_num<<endl;

    triangle *ts = new triangle[f_num];
    old_get_coordinates(ch,ts);
    std::cout<<"get_coordinates() done"<<std::endl;

    int id1,id2;
    Vector3d V1,V2;
    double max_dist = find_pinned_vertices(ch, id1, id2, V1, V2);
    std::cout<<"find_pinned_vertices() done"<<std::endl;

    apply(ch,ts,id1,id2,V1,V2,UV);

    double u_min=VERY_BIG_NUMBER, v_min=VERY_BIG_NUMBER, u_max=-VERY_BIG_NUMBER, v_max=-VERY_BIG_NUMBER;
    for(int i=0; i<v_num; i++) {
	    u_min = std::min(u_min, UV[i][0]);
	    u_max = std::max(u_max, UV[i][0]);
	    v_min = std::min(v_min, UV[i][1]);
	    v_max = std::max(v_max, UV[i][1]);	    
	}
	double len = std::max(u_max-u_min,v_max-v_min);
    for(int i=0; i<v_num; i++) {
	    UV[i][0] = (UV[i][0]-u_min)/len*max_dist;
	    UV[i][1] = (UV[i][1]-v_min)/len*max_dist;
	}
}

void eigen_parameterize(const chart &ch, UV_list &UV) {
    const aiMesh *mesh = ch.mi.mesh;
    int f_num = ch.faces.size();
    int v_num = ch.vertices.size();
    cout<<"f_num: "<<f_num<<" "<<"v_num: "<<v_num<<endl;

    triangle *ts = new triangle[f_num];
    get_coordinates(ch,ts);
    std::cout<<"get_coordinates() done"<<std::endl;

    int id1,id2;
    Vector3d V1,V2;
    double max_dist = find_pinned_vertices(ch, id1, id2, V1, V2);
    std::cout<<"find_pinned_vertices() done"<<std::endl;

    typedef Triplet<double> T;
    std::vector<T> A_tripletList, b_left_tripletList;
    for(int f=0; f<ch.faces.size(); f++) {
        triangle t = ts[f];
        double dt = (t[0][0]*t[1][1] - t[0][1]*t[1][0]) + (t[1][0]*t[2][1] - t[1][1]*t[2][0]) + (t[2][0]*t[0][1] - t[2][1]*t[0][0]);
        double co = dt>0? 1/sqrt(dt) : -1/(sqrt(-dt));
        for(int i=0; i<3; i++) {
            int j = (i+1)%3;  int k = (i+2)%3;
            int v0 = ch.faces[f].indices[i];
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
    lscg.setMaxIterations(v_num/2); //default is twice the number of columns of the matrix
    lscg.compute(A);
    std::cout<<"compute(A) done"<<std::endl;
    VectorXd X = lscg.solve(b); /* X is 2*(v_num-2) X 1 */
    std::cout<<"X done"<<std::endl;

    UV = new Vector2d[v_num];
    UV[id1] = Vector2d(-max_dist/2,-max_dist/2);
    UV[id2] = Vector2d(max_dist/2,max_dist/2);
    int UVid = 0;
    for(int index = 0; index<v_num-2; index++) {
        while(UVid==id1 || UVid==id2) {
            std::cout<<UVid<<":("<<UV[UVid].x()<<") ("<<UV[UVid].y()<<")"<<std::endl;
            UVid++;
        }
        UV[UVid] = Vector2d(X(index),X(v_num-2+index));
        std::cout<<UVid<<":("<<UV[UVid].x()<<") ("<<UV[UVid].y()<<")"<<std::endl;
        UVid++;
    }
}

void scene_parameterize (const scene_info &si, chart_list &all_charts, scene_UV_list &SUVL, param_mode mode) {
    if(SUVL.size()>0 || all_charts.size()==0)
        return;
    if(mode==PARAM_OPENNL)
        cout<<"Use OpenNL to solve"<<endl;
    else if(mode==PARAM_EIGEN)
        cout<<"Use Eigen to solve"<<endl;
    else
        return;

    for(chart_list::const_iterator iter=all_charts.begin(); iter!=all_charts.end(); iter++) {
        cout<<"chart "<<iter-all_charts.begin()<<" :"<<endl;
        UV_list UV;
        if(mode==PARAM_OPENNL)
            opennl_parameterize(*iter,UV);
        else if(mode==PARAM_EIGEN)
            eigen_parameterize(*iter,UV);
        SUVL.push_back(UV);
    }
}