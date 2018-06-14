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
    friend bool operator < (const edge& eA, const edge& eB) {
        if(eA.pA<eB.pA)
            return true;
        else if(eA.pA==eB.pA && eA.pB<eB.pB)
            return true;
        return false;
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

typedef std::map<edge,face_pair> edge_face_map;
typedef edge_face_map mesh_edges; /* edges of a mesh */
typedef std::vector<mesh_edges*> scene_edges; /* edges of the whole scene */

/* -------------------------------------------------------------------------------- */

extern void gen_edges (const aiScene *sc, scene_edges all_edges);
extern void segmentation (const aiScene *sc, scene_edges all_edges);

#endif

