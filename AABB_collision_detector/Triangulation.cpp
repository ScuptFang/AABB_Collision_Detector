#include "Triangulation.h"

namespace {
    bool pnpoly(int nvert, double* vertx, double* verty, double testx, double testy) {
        int i, j; bool c = false;
        for (i = 0, j = nvert - 1; i < nvert; j = i++) {
            if (((verty[i] > testy) != (verty[j] > testy)) && (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
                c = !c;
        }
        return c;
    }
    bool pnpoly(const std::vector<scu::Vector2>& vertex, const scu::Vector2& testp) {
        int nvert = vertex.size();
        int i, j; bool c = 0;
        for (i = 0, j = nvert - 1; i < nvert; j = i++) {
            if (((vertex[i][1] > testp[1]) != (vertex[j][1] > testp[1])) && (testp[0] < (vertex[j][0] - vertex[i][0]) * (testp[1] - vertex[i][1]) / (vertex[j][1] - vertex[i][1]) + vertex[i][0]))
                c = !c;
        }
        return c;
    }
}

void scu::Triangulation::GetIndexFromVertexLoop(const std::vector<Vertex>& loop, std::vector<Face>& faces, const scu::Vector3& normal)
{
    unsigned int vertexNum = loop.size();
    if (vertexNum < 3) return;
    unsigned int v0 = 0, v1 = vertexNum / 3, v2 = v1 + v1;
    bool hasNormal = normal.isNormalized();
    scu::Vector3 faceNormal;
    if (!hasNormal) {
        scu::Vector3 xAxis = (loop[v1] - loop[v0]).normalize();
        faceNormal = xAxis.cross((loop[v2] - loop[v0]).normalize()).normalize();
        scu::Vector3 yAxis = faceNormal.cross(xAxis).normalize();
    }
    else {
        faceNormal = normal;
    }
    std::vector<unsigned int> searchingIndex(vertexNum);
    for (unsigned int i = 0; i < vertexNum; i++) 
        searchingIndex[i] = i;
    
    int count = vertexNum;
    while (count >= 3) {
        std::vector<unsigned int> swapSearchingIndex;
        int l = count - 1; swapSearchingIndex.push_back(l);
        for (int m = 0, n = 1; n < count; m = n++) {
            scu::Vector3 judge = faceNormal.cross((loop[searchingIndex[m]] - loop[searchingIndex[l]]).normalize());
            double value = judge.dot((loop[searchingIndex[n]] - loop[searchingIndex[l]]));
            if (std::abs(value) <= std::numeric_limits<double>::min()) continue;
            if (value < 0.0) {
                swapSearchingIndex.push_back(searchingIndex[m]);
                l = m;
                continue;
            }
            std::vector<scu::Vector2> triangle =
            {
                scu::Vector2(loop[searchingIndex[l]][0], loop[searchingIndex[l]][1]),
                scu::Vector2(loop[searchingIndex[m]][0], loop[searchingIndex[m]][1]),
                scu::Vector2(loop[searchingIndex[n]][0], loop[searchingIndex[n]][1])
            }; bool isBlock = false;
            for (int i = 0; i < count; i++) {
                if (i == l || i == m || i == n) continue;
                isBlock = pnpoly(triangle, scu::Vector2(loop[searchingIndex[i]][0], loop[searchingIndex[i]][1]));
                if (isBlock) break;
            }
            if (isBlock) {
                swapSearchingIndex.push_back(searchingIndex[m]);
                l = m;
                continue;
            }
            Face f(searchingIndex[l], searchingIndex[m], searchingIndex[n]);
            faces.push_back(f);
        }
        swapSearchingIndex.swap(searchingIndex); count = searchingIndex.size();
    }
}