#include "Scene.h"

#include "Numeric/PCGSolver.h"

namespace asa
{
namespace
{
//////////////////////////////////////////////
// Add any custom classes or functions here //
//////////////////////////////////////////////

    struct Cell_ {
    Cell_(const Grid2 &grid, const Array2<Vector3> &data) : grid_(grid), data_(data) {}

    virtual Vector2 getIndex(const Vector2 &pos) = 0;

    Vector3 getValue(const Vector2 &pos)
    {
        const Vector2 ispos(getIndex(pos));
        const Vector2 isposmin(floorf(ispos.x), floorf(ispos.y));
        const Vector2 isposmax(ceilf(ispos.x), ceilf(ispos.y));
        const Vector2 t(ispos - isposmin);
        const Index2 mincorner((int)isposmin.x, (int)isposmin.y);
        const Index2 maxcorner((int)isposmax.x, (int)isposmax.y);

        const Index2 &size = data_.getSize();
        const Index2 id1(clamp(mincorner.x, 0, size.x - 1), clamp(mincorner.y, 0, size.y - 1));
        const Index2 id2(clamp(maxcorner.x, 0, size.x - 1), clamp(mincorner.y, 0, size.y - 1));
        const Index2 id3(clamp(mincorner.x, 0, size.x - 1), clamp(maxcorner.y, 0, size.y - 1));
        const Index2 id4(clamp(maxcorner.x, 0, size.x - 1), clamp(maxcorner.y, 0, size.y - 1));

        const Vector3 value = bilerp(data_[id1], data_[id2], data_[id3], data_[id4], t.x, t.y);
        return value;
    }

    Vector3 bilerp(const Vector3 aa, const Vector3 ba, const Vector3 ab, const Vector3 bb, const float tx, const float ty) {
        const Vector3 y1 = aa * (1.0f - tx) + ba * tx;
        const Vector3 y2 = ab * (1.0f - tx) + bb * tx;
        return y1 * (1.0f - ty) + y2 * ty;
    }

protected:
    const Grid2 &grid_;
    const Array2<Vector3> &data_;
};

struct Cell : Cell_ {
    Cell(const Grid2 &grid, const Array2<Vector3> &data) : Cell_(grid, data) { }

    Vector2 getIndex(const Vector2 &pos) {
        return grid_.getCellIndex(pos); 
    }
};


struct Face_ {
    Face_(const Grid2 &grid, const Array2<float> &data) : grid_(grid) , data_(data) { }

    virtual Vector2 getIndex(const Vector2 &pos) = 0;

    float getValue(const Vector2 &pos)
    {
        const Vector2 ispos(getIndex(pos));
        const Vector2 isposmin(floorf(ispos.x), floorf(ispos.y));
        const Vector2 isposmax(ceilf(ispos.x), ceilf(ispos.y));
        const Vector2 t(ispos - isposmin);
        const Index2 mincorner((int)isposmin.x, (int)isposmin.y);
        const Index2 maxcorner((int)isposmax.x, (int)isposmax.y);

        const Index2 &size = data_.getSize();
        const Index2 id1(clamp(mincorner.x, 0, size.x - 1), clamp(mincorner.y, 0, size.y - 1));
        const Index2 id2(clamp(maxcorner.x, 0, size.x - 1), clamp(mincorner.y, 0, size.y - 1));
        const Index2 id3(clamp(mincorner.x, 0, size.x - 1), clamp(maxcorner.y, 0, size.y - 1));
        const Index2 id4(clamp(maxcorner.x, 0, size.x - 1), clamp(maxcorner.y, 0, size.y - 1));

        const float value = bilerp(data_[id1], data_[id2], data_[id3], data_[id4], t.x, t.y);
        return value;
    }

    float bilerp(const float aa, const float ba, const float ab, const float bb, const float tx, const float ty)
    {
        const float y1 = aa * (1.0f - tx) + ba * tx;
        const float y2 = ab * (1.0f - tx) + bb * tx;
        return y1 * (1.0f - ty) + y2 * ty;
    }

protected:
    const Grid2 &grid_;
    const Array2<float> &data_;
};


struct Face : Face_ {
    Face(const Grid2 &grid, const Array2<float> &data, const unsigned int axis)
        : Face_(grid, data)
        , axis_(axis)
    {
    }

    Vector2 getIndex(const Vector2 &pos) {
        return grid_.getFaceIndex(pos, axis_);
    }

    const int axis_;
};
}  // namespace

    // advection
    void Fluid2::fluidAdvection(const float dt)
    {
        {
            Array2<Vector3> ink(inkRGB);
            Cell inkCell(grid, ink);
            const Index2 &size = ink.getSize();
            for (int i = 0; i < size.x; i++)
                for (int j = 0; j < size.y; j++) {
                    const Index2 id(i, j);

                    Vector2 pos(grid.getCellPos(id));
                    Vector2 vel((velocityX[id] + velocityX[Index2(i + 1, j)]), (velocityY[id] + velocityY[Index2(i, j + 1)]));
                    Vector2 endpos(pos - dt * vel);

                    inkRGB[id] = inkCell.getValue(endpos);

                }
        }
        
        {
            Array2<float> u(velocityX);
            Array2<float> v(velocityY);
            Face us(grid, u, 0);
            Face vs(grid, v, 1);
            const Index2 &sizeu = v.getSize();
            const Index2 &sizev = velocityY.getSize();

            for (int i = 0; i < sizeu.x; i++)
                for (int j = 0; j < sizeu.y; j++) {
                    const Index2 id(i, j);
                    const Index2 idv1(clamp(i - 1, 0, sizev.x - 1), clamp(j, 0, sizev.y - 1));
                    const Index2 idv2(clamp(i, 0, sizev.x - 1), clamp(j, 0, sizev.y - 1));
                    const Index2 idv3(clamp(i - 1, 0, sizev.x - 1), clamp(j + 1, 0, sizev.y - 1));
                    const Index2 idv4(clamp(i, 0, sizev.x - 1), clamp(j + 1, 0, sizev.y - 1));

                    const Vector2 pos(grid.getFaceXPos(id));
                    const Vector2 vel(u[id], (v[idv1] + v[idv2] + v[idv3] + v[idv4]) * 0.25f);
                    const Vector2 endpos(pos - dt * vel);

                    velocityX[id] = us.getValue(endpos);
                }

            for (int i = 0; i < sizev.x; i++)
                for (int j = 0; j < sizev.y; j++) {
                    const Index2 id(i, j);
                    const Index2 idu1(clamp(i, 0, sizeu.x - 1), clamp(j - 1, 0, sizeu.y - 1));
                    const Index2 idu2(clamp(i, 0, sizeu.x - 1), clamp(j, 0, sizeu.y - 1));
                    const Index2 idu3(clamp(i + 1, 0, sizeu.x - 1), clamp(j - 1, 0, sizeu.y - 1));
                    const Index2 idu4(clamp(i + 1, 0, sizeu.x - 1), clamp(j, 0, sizeu.y - 1));

                    const Vector2 pos(grid.getFaceYPos(id));
                    const Vector2 vel((u[idu1] + u[idu2] + u[idu3] + u[idu4]) * 0.25f, v[id]);
                    const Vector2 endpos(pos - dt * vel);

                    velocityY[id] = vs.getValue(endpos);
                }
        }
    }

    void Fluid2::fluidEmission()
    {
        if (Scene::testcase >= Scene::SMOKE) {
            AABox2 source(-0.1f, -1.9f, 0.1f, -1.7f);
            Vector2 ismin = grid.getCellIndex(source.minPosition);
            Vector2 ismax = grid.getCellIndex(source.maxPosition);
            Index2 bMin((int)floor(ismin.x), (int)floor(ismin.y));
            Index2 bMax((int)ceil(ismax.x), (int)ceil(ismax.y));

            for (int i = bMin.x; i <= bMax.x; i++)
                for (int j = bMin.y; j <= bMax.y; j++)
                    if (i < (bMin.x + bMax.x) / 2)
                        inkRGB[Index2(i, j)] = Vector3(1, 1, 0);
                    else if (i > (bMin.x + bMax.x) / 2)
                        inkRGB[Index2(i, j)] = Vector3(1, 0, 1);
                    else
                        inkRGB[Index2(i, j)] = Vector3(1, 1, 1);

            for (int i = bMin.x; i <= bMax.x + 1; i++)
                for (int j = bMin.y; j <= bMax.y; j++)
                    velocityX[Index2(i, j)] = 0.0f;

            for (int i = bMin.x; i <= bMax.x; i++)
                for (int j = bMin.y; j <= bMax.y + 1; j++)
                    velocityY[Index2(i, j)] = 8.0f;
        }
    }

    void Fluid2::fluidVolumeForces(const float dt)
    {
        if (Scene::testcase >= Scene::SMOKE) {
            const float dtGravity = dt * Scene::kGravity;
            const Index2 &size = velocityY.getSize();
            for (int i = 0, n = size.x * size.y; i < n; i++)
                velocityY[i] += dtGravity;
        }
    }

    void Fluid2::fluidViscosity(const float dt)
    {
        if (Scene::testcase >= Scene::SMOKE) {
            // Viscosity term HERE
            Array2<float> ucopy(velocityX);
            Array2<float> vcopy(velocityY);
            const Index2 &sizeu = velocityX.getSize();
            const Index2 &sizev = velocityY.getSize();

            const Vector2 dx = grid.getCellDx();
            const Vector2 invDxSq(1.0f / (dx.x * dx.x), 1.0f / (dx.y * dx.y));
            const float dtMuOverRho = dt * Scene::kViscosity / Scene::kDensity;

            for (int i = 0; i < sizeu.x; i++)
                for (int j = 0; j < sizeu.y; j++) {
                    const Index2 id(i, j);
                    const Index2 id1(clamp(i - 1, 0, sizeu.x - 1), j);
                    const Index2 id2(clamp(i + 1, 0, sizeu.x - 1), j);
                    const Index2 id3(i, clamp(j - 1, 0, sizeu.y - 1));
                    const Index2 id4(i, clamp(j + 1, 0, sizeu.y - 1));
                    velocityX[id] += dtMuOverRho * ((ucopy[id1] - 2.0f * ucopy[id] + ucopy[id2]) * invDxSq.x + (ucopy[id3] - 2.0f * ucopy[id] + ucopy[id4]) * invDxSq.y);
                }

            for (int i = 0; i < sizev.x; i++)
                for (int j = 0; j < sizev.y; j++) {
                    const Index2 id(i, j);
                    const Index2 id1(clamp(i - 1, 0, sizev.x - 1), j);
                    const Index2 id2(clamp(i + 1, 0, sizev.x - 1), j);
                    const Index2 id3(i, clamp(j - 1, 0, sizev.y - 1));
                    const Index2 id4(i, clamp(j + 1, 0, sizev.y - 1));
                    velocityY[id] += dtMuOverRho * ((vcopy[id1] - 2.0f * vcopy[id] + vcopy[id2]) * invDxSq.x + (vcopy[id3] - 2.0f * vcopy[id] + vcopy[id4]) * invDxSq.y);
                }
        }
    }

    void Fluid2::fluidPressureProjection(const float dt)
    {
        if (Scene::testcase >= Scene::SMOKE) {
            const Vector2 dx = grid.getCellDx();
            const Vector2 invDx(1.0f / dx.x, 1.0f / dx.y);
            const Vector2 invDxSq(1.0f / (dx.x * dx.x), 1.0f / (dx.y * dx.y));

            const Index2 &size = pressure.getSize();
            const Index2 &sizeu = velocityX.getSize();
            const Index2 &sizev = velocityY.getSize();

            for (int j = 0; j < sizeu.y; j++) {
                velocityX[Index2(0, j)] = 0.0f;
                velocityX[Index2(sizeu.x - 1, j)] = 0.0f;
            }
            for (int i = 0; i < sizev.x; i++) {
                velocityY[Index2(i, 0)] = 0.0f;
            }

            const float rhoOverDt = Scene::kDensity / dt;
            std::vector<double> rhs(size.x * size.y);
            for (int i = 0; i < size.x; i++)
                for (int j = 0; j < size.y; j++) {
                    const Index2 id(i, j);
                    rhs[pressure.getLinearIndex(i, j)] = -rhoOverDt * ((velocityX[Index2(i + 1, j)] - velocityX[id]) * invDx.x + (velocityY[Index2(i, j + 1)] - velocityY[id]) * invDx.y);
                }


            SparseMatrix<double> A(size.x * size.y, 5);
            for (int i = 0; i < size.x; i++)
                for (int j = 0; j < size.y; j++) {
                    const int id = pressure.getLinearIndex(i, j);
                    if (i > 0) {
                        const int id1 = pressure.getLinearIndex(i - 1, j);
                        A.add_to_element(id, id, 1. * invDxSq.x);
                        A.add_to_element(id, id1, -1. * invDxSq.x);
                    }
                    if (i < size.x - 1) {
                        const int id1 = pressure.getLinearIndex(i + 1, j);
                        A.add_to_element(id, id, 1. * invDxSq.x);
                        A.add_to_element(id, id1, -1. * invDxSq.x);
                    }
                    if (j > 0) {
                        const int id1 = pressure.getLinearIndex(i, j - 1);
                        A.add_to_element(id, id, 1. * invDxSq.y);
                        A.add_to_element(id, id1, -1. * invDxSq.y);
                    }
                    A.add_to_element(id, id, 1. * invDxSq.y);
                    if (j < size.y - 1) {
                        const int id1 = pressure.getLinearIndex(i, j + 1);
                        A.add_to_element(id, id1, -1. * invDxSq.y);
                    }
                }


            PCGSolver<double> solver;
            solver.set_solver_parameters(1e-6, 10000);
            std::vector<double> result(size.x * size.y);
            double residual_out;
            int iterations_out;
            solver.solve(A, rhs, result, residual_out, iterations_out);  // Tarda mucho en ejecutar - OGM

            for (int i = 0, n = size.x * size.y; i < n; i++)
                pressure[i] = (float)result[i];


            const float dtOverRho = dt / Scene::kDensity;
            for (int i = 1; i < sizeu.x - 1; i++)
                for (int j = 0; j < sizeu.y; j++) {
                    const Index2 id(i, j);
                    const float gradp = (pressure[id] - pressure[Index2(i - 1, j)]) * invDx.x;
                    velocityX[id] -= dtOverRho * gradp;
                }
            for (int i = 0; i < sizev.x; i++)
                for (int j = 1; j < sizev.y - 1; j++) {
                    const Index2 id(i, j);
                    const float gradp = (pressure[id] - pressure[Index2(i, j - 1)]) * invDx.y;
                    velocityY[id] -= dtOverRho * gradp;
                }

            for (int i = 0; i < sizev.x; i++) {
                const Index2 id(i, sizev.y - 1);
                const float gradp = (0.0f - pressure[Index2(i, size.y - 1)]) * invDx.y;
                velocityY[id] -= dtOverRho * gradp;
            }
        }
    }
}  // namespace asa
