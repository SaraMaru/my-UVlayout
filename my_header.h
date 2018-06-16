#ifndef MY_HEADER_H
#define MY_HEADER_H

#include <stdlib.h>
#include <stdio.h>
#include <map>
#include <vector>
#include <GL/glut.h>
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <FreeImage.h> /* for grab() */

/* -------------------------------------------------------------------------------- */

enum edge_lock { /* can not connect to a locked point */
    NO_LOCK, LOCK_A, LOCK_B, LOCK_BOTH
};

enum link_type {
    NO_LINK, LINK_AA, LINK_AB, LINK_BA, LINK_BB
};

struct edge {
    int pA,pB;
    edge(int a, int b) { /* important for find() and < */
        if(a<b) {
            pA = a; pB =b;
        }
        else {
            pA = b; pB = a;
        }
    }
    friend bool operator == (const edge& eA, const edge& eB) {
        if(eA.pA==eB.pA && eA.pB==eB.pB)
            return true;
        return false;
    }
    friend bool operator != (const edge& eA, const edge& eB) {
        return !(eA==eB);
    }
    friend bool operator < (const edge& eA, const edge& eB) {
        if(eA.pA<eB.pA)
            return true;
        else if(eA.pA==eB.pA && eA.pB<eB.pB)
            return true;
        return false;
    }
};

struct seg_edge {
    edge e;
    float sharpness; //angle
    bool isChosen;
    bool islockedA;
    bool islockedB;

    seg_edge(edge ee) : e(ee) {
        isChosen = islockedA = islockedB = false;
    }
    seg_edge(edge ee, float s) : e(ee) {
        sharpness = s;
        isChosen = islockedA = islockedB = false;
    }
    link_type canLinkTo(const seg_edge &se) {
        if(se.islockedA || se.islockedB) /* notice the || here (not &&) */
            return NO_LINK;
        if(islockedA && islockedB)
            return NO_LINK;
        if(!se.islockedA) {
            if(e.pA==se.e.pA && !islockedA)
                return LINK_AA;
            else if(e.pB==se.e.pA && !islockedB)
                return LINK_BA;
        }
        if(!se.islockedB) {
            if(e.pA==se.e.pB && !islockedA)
                return LINK_AB;
            else if(e.pB==se.e.pB && !islockedB)
                return LINK_BB;
        }
        return NO_LINK;
    }
};

struct face_pair {
    int fA,fB;
    face_pair(int a, int b) : fA(a), fB(b) {}
    face_pair(int a) : fA(a), fB(-1) {}
    void insert(int b) {
        if(fB!=-1) {
            printf("ERROR:Only 2 faces can be adjacent to one edge!\n");
        }
        else {
            fB = b;
        } 
    }
};

struct angle_index {
    float angle;
    int id;
    angle_index(float a, int i) : angle(a), id(i) {}
    friend bool operator < (const angle_index& x, const angle_index& y) {
        if(x.angle<y.angle)
            return true;
        return false;
    }
};

/* -------------------------------------------------------------------------------- */

typedef std::map<edge,face_pair> edge_face_map; /* edges of a mesh */
typedef std::vector<edge_face_map> scene_edge_face_map; /* edges of the whole scene */

typedef aiVector3D* face_normals;
typedef std::vector<face_normals> scene_face_normals;

typedef std::vector<edge> edge_list;
typedef std::vector<seg_edge> seg_edge_list;
typedef std::vector<edge_list> scene_edge_list; 

/* -------------------------------------------------------------------------------- */

extern void gen_edges (const aiMesh* mesh, edge_face_map &efm);
extern void scene_segmentation (const aiScene *sc, scene_edge_face_map &all_edges, scene_edge_list &sce);

/* -------------------------------------------------------------------------------- */

#endif

