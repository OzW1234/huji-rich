__author__ = 'zmbq'

cdef extern from "tetgen.h":
    cdef cppclass tetgenio:  # Documented in tetgen.h
        tetgenio();

        ctypedef struct pointparam:
            double uv[2];
            int tag;
            int type;

        ctypedef struct polygon:
            int *vertexlist;
            int numberofvertices;

        ctypedef struct facet:
            polygon *polygonlist;
            int numberofpolygons;
            double *holelist;
            int numberofholes;

        ctypedef struct voroedge:
            int v1, v2;
            double vnormal[3];

        ctypedef struct vorofacet:
            int c1, c2;
            int *elist;

        int firstnumber;
        int mesh_dim;
        int useindex;

        double *pointlist;
        double *pointattributelist;
        double *pointmtrlist;
        int  *pointmarkerlist;
        pointparam *pointparamlist;
        int numberofpoints;
        int numberofpointattributes;
        int numberofpointmtrs;

        int  *tetrahedronlist;
        double *tetrahedronattributelist;
        double *tetrahedronvolumelist;
        int  *neighborlist;
        int numberoftetrahedra;
        int numberofcorners;
        int numberoftetrahedronattributes;

        int  *tetrahedronlist;
        double *tetrahedronattributelist;
        double *tetrahedronvolumelist;
        int  *neighborlist;
        int numberoftetrahedra;
        int numberofcorners;
        int numberoftetrahedronattributes;

        facet *facetlist;
        int *facetmarkerlist;
        int numberoffacets;

        double *holelist;
        int numberofholes;

        double *regionlist;
        int numberofregions;

        double *facetconstraintlist;
        int numberoffacetconstraints;

        double *segmentconstraintlist;
        int numberofsegmentconstraints;

        int *trifacelist;
        int *trifacemarkerlist;
        int *o2facelist;
        int *adjtetlist;
        int numberoftrifaces;

        int *edgelist;
        int *edgemarkerlist;
        int *o2edgelist;
        int *edgeadjtetlist;
        int numberofedges;

        double *vpointlist;
        voroedge *vedgelist;
        vorofacet *vfacetlist;
        int **vcelllist;
        int numberofvpoints;
        int numberofvedges;
        int numberofvfacets;
        int numberofvcells;

        # Extra members (mainly callback functions) ignored
        #void *geomhandle;
        #GetVertexParamOnEdge getvertexparamonedge;
        #GetSteinerOnEdge getsteineronedge;
        #GetVertexParamOnFace getvertexparamonface;
        #GetEdgeSteinerParamOnFace getedgesteinerparamonface;
        #GetSteinerOnFace getsteineronface;
        #
        #TetSizeFunc tetunsuitable;

cdef extern from "tetgen.h":
    void tetrahedralize(char *switches, tetgenio *input, tetgenio *output,
                        tetgenio *addin = NULL, tetgenio *bgmin = NULL);
