#include "Scene.h"
#include "Std/CMath.h"
#include "Numeric/PCGSolver.h"

namespace asa
{
namespace
{
inline int clamp(int x, int a, int b)
{
    return x < a ? a : (x > b ? b : x);
}

inline Vector2 clamp(Vector2 x, Vector2 a, Vector2 b)
{
    float aux1 = x.x < a.x ? a.x : (x.x > b.x ? b.x : x.x);
    float aux2 = x.y < a.y ? a.y : (x.y > b.y ? b.y : x.y);
    return Vector2(aux1, aux2);
}

inline float getRandom(const float value1, const float value2)
{
    const float aux = (float)rand() / (float)RAND_MAX;
    const float dif = fabs(value1 - value2);
    return (aux * dif) + fmin(value1, value2);
}

template <typename T>
inline T bilerp(const Vector2 pos, Array2<T> &field, const Index2 size)
{
    const Vector2 posMin(floorf(pos.x), floorf(pos.y));
    const Vector2 posMax(ceilf(pos.x), ceilf(pos.y));
    const Vector2 t(pos - posMin);

    const Index2 id1(clamp((int)posMin.x, 0, size.x - 1), clamp((int)posMin.y, 0, size.y - 1));
    const Index2 id2(clamp((int)posMax.x, 0, size.x - 1), clamp((int)posMin.y, 0, size.y - 1));
    const Index2 id3(clamp((int)posMin.x, 0, size.x - 1), clamp((int)posMax.y, 0, size.y - 1));
    const Index2 id4(clamp((int)posMax.x, 0, size.x - 1), clamp((int)posMax.y, 0, size.y - 1));

    const T y1 = field[id1] * (1.0f - t.x) + field[id2] * t.x;
    const T y2 = field[id3] * (1.0f - t.x) + field[id4] * t.x;
    return y1 * (1 - t.y) + y2 * t.y;
}

struct BaseSampler {
    BaseSampler(const Grid2 &grid, const Array2<float> &data)
        : grid_(grid)
        , data_(data)
    {
    }

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

struct CellSampler : BaseSampler {
    CellSampler(const Grid2 &grid, const Array2<float> &data)
        : BaseSampler(grid, data)
    {
    }

    Vector2 getIndex(const Vector2 &pos) { return grid_.getCellIndex(pos); }
};

struct FaceSampler : BaseSampler {
    FaceSampler(const Grid2 &grid, const Array2<float> &data, const unsigned int axis)
        : BaseSampler(grid, data)
        , axis_(axis)
    {
    }

    Vector2 getIndex(const Vector2 &pos) { return grid_.getFaceIndex(pos, axis_); }

    const unsigned int axis_;

};

//////////////////////////////////////////////
// Add any custom classes or functions here //
//////////////////////////////////////////////

Vector3 operator/(const Vector3 v, const float &k)
{
    return Vector3(v.x * k, v.y * k, v.z * k);
}

void accumulateCell(Array2<Vector3> &vect, Array2<float> &weights, Vector3 val, int i, int j, float fx, float fy)
{
    float weight;

    if ((i >= 0 && i < vect.getSize().x) && (j >= 0 && j < vect.getSize().y)) {
        weight = (1 - fx) * (1 - fy);
        vect.setValue(Index2(i, j), vect.getValue(Index2(i, j)) + val.operator*(weight));
        weights.setValue(Index2(i, j), weights.getValue(Index2(i, j)) + weight);
    }

    if ((i + 1 >= 0 && i + 1 < vect.getSize().x) && (j >= 0 && j < vect.getSize().y)) {
        weight = fx * (1 - fy);
        vect.setValue(Index2(i + 1, j), vect.getValue(Index2(i + 1, j)) + val.operator*(weight));
        weights.setValue(Index2(i + 1, j), weights.getValue(Index2(i + 1, j)) + weight);
    }

    if ((i >= 0 && i < vect.getSize().x) && (j + 1 >= 0 && j + 1 < vect.getSize().y)) {
        weight = (1 - fx) * fy;
        vect.setValue(Index2(i, j + 1), vect.getValue(Index2(i, j + 1)) + val.operator*(weight));
        weights.setValue(Index2(i, j + 1), weights.getValue(Index2(i, j + 1)) + weight);
    }

    if ((i + 1 >= 0 && i + 1 < vect.getSize().x) && (j + 1 >= 0 && j + 1 < vect.getSize().y)) {
        weight = fx * fy;
        vect.setValue(Index2(i + 1, j + 1), vect.getValue(Index2(i + 1, j + 1)) + val.operator*(weight));
        weights.setValue(Index2(i + 1, j + 1), weights.getValue(Index2(i + 1, j + 1)) + weight);
    }
}
void accumulateCell(Array2<float> &vect, Array2<float> &weights, float val, int i, int j, float fx, float fy)
{
    float weight;

    if ((i >= 0 && i < vect.getSize().x) && (j >= 0 && j < vect.getSize().y)) {
        weight = (1 - fx) * (1 - fy);
        vect.setValue(Index2(i, j), vect.getValue(Index2(i, j)) + weight * val);
        weights.setValue(Index2(i, j), weights.getValue(Index2(i, j)) + weight);
    }

    if ((i + 1 >= 0 && i + 1 < vect.getSize().x) && (j >= 0 && j < vect.getSize().y)) {
        weight = fx * (1 - fy);
        vect.setValue(Index2(i + 1, j), vect.getValue(Index2(i + 1, j)) + weight * val);
        weights.setValue(Index2(i + 1, j), weights.getValue(Index2(i + 1, j)) + weight);
    }

    if ((i >= 0 && i < vect.getSize().x) && (j + 1 >= 0 && j + 1 < vect.getSize().y)) {
        weight = (1 - fx) * fy;
        vect.setValue(Index2(i, j + 1), vect.getValue(Index2(i, j + 1)) + weight * val);
        weights.setValue(Index2(i, j + 1), weights.getValue(Index2(i, j + 1)) + weight);
    }

    if ((i + 1 >= 0 && i + 1 < vect.getSize().x) && (j + 1 >= 0 && j + 1 < vect.getSize().y)) {
        weight = fx * fy;
        vect.setValue(Index2(i + 1, j + 1), vect.getValue(Index2(i + 1, j + 1)) + weight * val);
        weights.setValue(Index2(i + 1, j + 1), weights.getValue(Index2(i + 1, j + 1)) + weight);
    }
}
/// Control emission variable
int count = 0;

/// Weights grid
// Array2 <float> weights(grid.getSize().x + 1, grid.getSize().y + 1);
}  // namespace
   // namespace

// init particles
void Fluid2::initParticles()
{
    // particle sampling on the entire domain
    // Get size of grid
    const Index2 &size = grid.getSize();
    // Get dx and dy which is the dif between the center and the border of the cell
    const float dx =
        grid.getCellDx().x * 0.5; /* (grid.getDomain().maxPosition.x - grid.getDomain().minPosition.x) / (size.x * 2);*/
    const float dy =
        grid.getCellDx().y * 0.5; /* (grid.getDomain().maxPosition.y - grid.getDomain().minPosition.y) / (size.y * 2);*/
    for (unsigned int i = 0; i < size.x; ++i) {
        for (unsigned int j = 0; j < size.y; ++j) {
            // For each cell get index and position
            const Index2 id(i, j);
            const Vector2 posCenter(grid.getCellPos(id));
            // Divide the cell in four, and get random position for each partition
            const Vector2 pos1(getRandom(posCenter.x - dx, posCenter.x), getRandom(posCenter.y - dy, posCenter.y));
            const Vector2 pos2(getRandom(posCenter.x + dx, posCenter.x), getRandom(posCenter.y - dy, posCenter.y));
            const Vector2 pos3(getRandom(posCenter.x - dx, posCenter.x), getRandom(posCenter.y + dy, posCenter.y));
            const Vector2 pos4(getRandom(posCenter.x + dx, posCenter.x), getRandom(posCenter.y + dy, posCenter.y));
            // Add a particle in all the partitions by cell
            particles.addParticle(pos1);
            particles.addParticle(pos2);
            particles.addParticle(pos3);
            particles.addParticle(pos4);
        }
    }
}

// advection
void Fluid2::fluidAdvection(const float dt)
{
    if (flipEnabled) {
        // move particles with RK2 with grid velocities
        for (unsigned int i = 0; i < particles.getSize(); ++i) {
            /// foreach particle search the index cell
            const Vector2 &posParticle = particles.getPosition(i);
            Vector2 auxVel;

            // first stage of Runge-Kutta 2 (do a half Euler step)
            auxVel.x = bilerp(grid.getFaceIndex(posParticle, 0), velocityX, velocityX.getSize());
            auxVel.y = bilerp(grid.getFaceIndex(posParticle, 1), velocityY, velocityY.getSize());
            const Vector2 midPos = posParticle + 0.5 * dt * auxVel;

            // second stage of Runge-Kutta 2
            auxVel.x = bilerp(grid.getFaceIndex(midPos, 0), velocityX, velocityX.getSize());
            auxVel.y = bilerp(grid.getFaceIndex(midPos, 1), velocityY, velocityY.getSize());
            const Vector2 posFinal = posParticle + dt * auxVel;

            particles.setPosition(i, posFinal);

            // particles.setInk(i, bilerp(grid.getCellIndex(posFinal), ink, ink.getSize()));
        }

        // ensure particle remains inside the domain
        for (unsigned int i = 0; i < particles.getSize(); ++i) {
            particles.setPosition(
                i, clamp(particles.getPosition(i), grid.getDomain().minPosition, grid.getDomain().maxPosition));
        }

        // create ink grid from particles
        Array2<float> weights(grid.getSize().x + 1, grid.getSize().y + 1);
        weights.clear();
        inkRGB.clear();
        for (int i = 0; i < particles.getSize(); ++i) {
            const Vector2 &posParticle = particles.getPosition(i);
            const Vector2 inkVal = grid.getCellIndex(posParticle);
            // Assign weights for each node
            accumulateCell(inkRGB,
                           weights,
                           particles.getInk(i),
                           (int)inkVal.x,
                           (int)inkVal.y,
                           inkVal.x - floorf(inkVal.x),
                           inkVal.y - floorf(inkVal.y));
        }
        /// Normalize weights
        for (unsigned int j = 0; j < inkRGB.getSize().y; ++j) {
            for (unsigned int i = 0; i < inkRGB.getSize().x; ++i) {
                const Index2 id = Index2(i, j);
                if (weights.getValue(id) != 0)
                    inkRGB.setValue(id, operator/(inkRGB.getValue(id), weights.getValue(id)));
            }
        }

        // create velocityX grid from particles
        weights.clear();
        velocityX.clear();
        for (unsigned int i = 0; i < particles.getSize(); ++i) {
            const Vector2 &posParticle = particles.getPosition(i);
            const Vector2 uVel = grid.getFaceIndex(posParticle, 0);
            // Assign weights for each node
            accumulateCell(velocityX,
                           weights,
                           particles.getVelocity(i).x,
                           (int)uVel.x,
                           (int)uVel.y,
                           uVel.x - floorf(uVel.x),
                           uVel.y - floorf(uVel.y));
        }
        /// Normalize weights
        for (unsigned int j = 0; j < velocityX.getSize().y; ++j) {
            for (unsigned int i = 0; i < velocityX.getSize().x; ++i) {
                const Index2 id = Index2(i, j);
                if (weights.getValue(id) != 0)
                    velocityX.setValue(id, velocityX.getValue(id) / weights.getValue(id));
            }
        }

        // create velocityY grid from particles
        weights.clear();
        velocityY.clear();
        for (unsigned int i = 0; i < particles.getSize(); ++i) {
            const Vector2 &posParticle = particles.getPosition(i);
            const Vector2 vVel = grid.getFaceIndex(posParticle, 1);
            // Assign weights for each node
            accumulateCell(velocityY,
                           weights,
                           particles.getVelocity(i).y,
                           (int)vVel.x,
                           (int)vVel.y,
                           vVel.x - floorf(vVel.x),
                           vVel.y - floorf(vVel.y));
        }
        /// Normalize weights
        for (unsigned int j = 0; j < velocityY.getSize().y; ++j) {
            for (unsigned int i = 0; i < velocityY.getSize().x; ++i) {
                const Index2 id = Index2(i, j);
                if (weights.getValue(id) != 0)
                    velocityY.setValue(id, velocityY.getValue(id) / weights.getValue(id));
            }
        }

        // save current state velocities
        oldVelocityX.copy(velocityX);
        oldVelocityY.copy(velocityY);
    } else {
        // ink advection
        {
            Array2<Vector3> inkAux(inkRGB);
            const Index2 sizeInk = inkRGB.getSize();
            for (unsigned int i = 0; i < sizeInk.x; ++i) {
                for (unsigned int j = 0; j < sizeInk.y; ++j) {
                    const Index2 id(i, j);
                    /// Calculate global coordinates
                    const Vector2 pos(grid.getCellPos(id));
                    /// Calculate advencion method
                    const Vector2 vel((velocityX[id] + velocityX[Index2(i + 1, j)]) * 0.5f,
                                   (velocityY[id] + velocityY[Index2(i, j + 1)]) * 0.5f);
                    const Vector2 endpos(pos - dt * vel);
                    /// Bilineal interpolation with index
                    inkRGB[id] = bilerp(grid.getCellIndex(endpos), inkAux, sizeInk);
                }
            }
        }

        // velocity advection
        {
            Array2<float> uAux(velocityX);
            Array2<float> vAux(velocityY);
            const Index2 sizeU = velocityX.getSize();
            const Index2 sizeV = velocityY.getSize();
            for (int i = 0; i < sizeU.x; ++i) {
                for (int j = 0; j < sizeU.y; ++j) {
                    const Index2 id(i, j);
                    /// Calculate index of the previus velocities in X to compute the velocityX
                    const Index2 id1(clamp(i - 1, 0, (int)sizeV.x - 1), clamp(j, 0, (int)sizeV.y - 1));
                    const Index2 id2(clamp(i, 0, (int)sizeV.x - 1), clamp(j, 0, (int)sizeV.y - 1));
                    const Index2 id3(clamp(i - 1, 0, (int)sizeV.x - 1), clamp(j + 1, 0, (int)sizeV.y - 1));
                    const Index2 id4(clamp(i, 0, (int)sizeV.x - 1), clamp(j + 1, 0, (int)sizeV.y - 1));
                    /// Calculate global coordinates VelocityX
                    const Vector2 pos(grid.getFaceXPos(id));
                    /// Compute advection methond
                    const Vector2 vel(uAux[id], (vAux[id1] + vAux[id2] + vAux[id3] + vAux[id4]) * 0.25f);
                    const Vector2 endpos(pos - dt * vel);
                    /// Bilineal interpolation with index, face index in axis 0, horizontal
                    velocityX[id] = bilerp(grid.getFaceIndex(endpos, 0), uAux, sizeU);
                }
            }
            for (unsigned int i = 0; i < sizeV.x; ++i) {
                for (unsigned int j = 0; j < sizeV.y; ++j) {
                    const Index2 id(i, j);
                    /// Calculate index of the previus velocities in X to compute the velocityX
                    const Index2 id1(clamp((int)i, 0, (int)sizeU.x - 1), clamp((int)j - 1, 0, (int)sizeU.y - 1));
                    const Index2 id2(clamp((int)i, 0, (int)sizeU.x - 1), clamp((int)j - 1, 0, (int)sizeU.y - 1));
                    const Index2 id3(clamp((int)i + 1, 0, (int)sizeU.x - 1), clamp((int)j, 0, (int)sizeU.y - 1));
                    const Index2 id4(clamp((int)i + 1, 0, (int)sizeU.x - 1), clamp((int)j, 0, (int)sizeU.y - 1));
                    /// Calculate global coordinates VelocityY
                    const Vector2 pos(grid.getFaceYPos(id));
                    /// Compute advection methond
                    const Vector2 vel((uAux[id1] + uAux[id2] + uAux[id3] + uAux[id4]) * 0.25f, vAux[id]);
                    const Vector2 endpos(pos - dt * vel);
                    /// Bilineal interpolation with index, face index in axis 1, vertical
                    velocityY[id] = bilerp(grid.getFaceIndex(endpos, 1), vAux, sizeV);
                }
            }
        }
    }
}

void Fluid2::fluidEmission()
{
    Vector2 vel(0, 6);
    AABox2 source(-0.1f, -1.9f, 0.1f, -1.7f);

    if (flipEnabled){
        // modify particles properties if inside the domain
        Vector2 ismin = grid.getCellIndex(source.minPosition);
        Vector2 ismax = grid.getCellIndex(source.maxPosition);
        Index2 bMin((int)floor(ismin.x), (int)floor(ismin.y));
        Index2 bMax((int)ceil(ismax.x), (int)ceil(ismax.y));

        for (int i = 0; i < particles.getSize(); ++i) {
            const Vector2 &posParticle = particles.getPosition(i);
            const Vector2 id = grid.getCellIndex(posParticle);
            if ((posParticle.x >= source.minPosition.x && posParticle.x <= source.maxPosition.x) &&
                (posParticle.y >= source.minPosition.y && posParticle.y <= source.maxPosition.y)) {
                if (posParticle.x < (source.minPosition.x + source.maxPosition.x) / 2)
                    particles.setInk(i, Vector3(1, 1, 0));
                else if (posParticle.x > (source.minPosition.x + source.maxPosition.x) / 2)
                    particles.setInk(i, Vector3(1, 0, 1));
                else
                    particles.setInk(i, Vector3(1, 1, 1));

                particles.setVelocity(i, vel);
            }
        }
    } else {
        const AABox2 source1(-0.2f, -0.2f, 0.2f, 0.2f);

        const Vector2 minCell1 = grid.getCellIndex(source1.minPosition);
        const Vector2 maxCell1 = grid.getCellIndex(source1.maxPosition);
        const Vector2 difCell1 = (maxCell1 - minCell1) / Vector2(2.0f, 2.0f);
        const Vector2 middleCell1 = difCell1 + minCell1;

        const Index2 bMin((int)floor(minCell1.x), (int)floor(minCell1.y));
        const Index2 bMax((int)ceil(maxCell1.x), (int)ceil(maxCell1.y));
        const Index2 middleMin1((int)floor(middleCell1.x), (int)floor(middleCell1.y));
        const Index2 middleMax1((int)ceil(middleCell1.x), (int)ceil(middleCell1.y));

        for (unsigned int i = bMin.x; i <= bMax.x; ++i) {
            for (unsigned int j = bMin.y; j <= bMax.y; ++j) {
                const Index2 id(i, j);
                if (j > bMin.y && j < bMax.y) {
                    if (i < middleMin1.x) {
                        inkRGB[id] = Vector3(1,0,1);
                        velocityX[id] = -5.0f;
                        velocityY[id] = 0.0f;
                    }
                    if (i > middleMax1.x) {
                        inkRGB[id] = Vector3(1,1,0);
                        velocityX[id] = 5.0f;
                        velocityY[id] = 0.0f;
                    }
                }
            }
        }
        /// Control de emission
        if (count++ % 20 < 13) {
            const AABox2 source2(-0.1f, -1.9f, 0.1f, -1.7f);

            const Vector2 minCell2 = grid.getCellIndex(source2.minPosition);
            const Vector2 maxCell2 = grid.getCellIndex(source2.maxPosition);

            const Index2 cornerMin2((int)floor(minCell2.x), (int)floor(minCell2.y));
            const Index2 cornerMax2((int)ceil(maxCell2.x), (int)ceil(maxCell2.y));

            for (int i = cornerMin2.x; i <= cornerMax2.x; ++i) {
                for (int j = cornerMin2.y; j <= cornerMax2.y; ++j) {
                    const Index2 id(i, j);
                    inkRGB[id] = Vector3(1,0,1);
                    velocityX[id] = 0.0f;
                    velocityY[id] = 10.0f;
                }
            }

            const AABox2 source3(-0.1f, 1.7f, 0.1f, 1.9f);

            const Vector2 minCell3 = grid.getCellIndex(source3.minPosition);
            const Vector2 maxCell3 = grid.getCellIndex(source3.maxPosition);

            const Index2 cornerMin3((int)floor(minCell3.x), (int)floor(minCell3.y));
            const Index2 cornerMax3((int)ceil(maxCell3.x), (int)ceil(maxCell3.y));

            for (int i = cornerMin3.x; i <= cornerMax3.x; ++i) {
                for (int j = cornerMin3.y; j <= cornerMax3.y; ++j) {
                    const Index2 id(i, j);
                    inkRGB[id] = Vector3 (1,0,1);
                    velocityX[id] = 0.0f;
                    velocityY[id] = -10.0f;
                }
            }
        }
    }
}

void Fluid2::fluidVolumeForces(const float dt)
{
    // gravity
    const float gravity = dt * Scene::kGravity;
    const Index2 sizeV = velocityY.getSize();
    for (unsigned int i = 0; i < sizeV.x; ++i) {
        for (unsigned int j = 0; j < sizeV.y; ++j) {
            const Index2 id(i, j);
            velocityY[id] += gravity;
        }
    }
}

void Fluid2::fluidViscosity(const float dt)
{
    // viscosity
    Array2<float> uAux(velocityX);
    Array2<float> vAux(velocityY);
    const Index2 sizeU = velocityX.getSize();
    const Index2 sizeV = velocityY.getSize();

    const Vector2 dx(grid.getCellDx());
    const Vector2 invDxPow(1.0f / pow(dx.x, 2), 1.0f / pow(dx.y, 2));
    const float dtVisDen = dt * Scene::kViscosity / Scene::kDensity;

    for (int i = 0; i < sizeU.x; ++i) {
        for (int j = 0; j < sizeU.y; ++j) {
            const Index2 id(i, j);
            /// Calculate index
            const Index2 id1(clamp(i + 1, 0, (int)sizeU.x - 1), j);
            const Index2 id2(clamp(i - 1, 0, (int)sizeU.x - 1), j);
            const Index2 id3(i, clamp(j + 1, 0, (int)sizeU.y - 1));
            const Index2 id4(i, clamp(j - 1, 0, (int)sizeU.y - 1));
            /// Compute viscosity in U
            velocityX[id] += dtVisDen * ((uAux[id1] - 2.0f * uAux[id] + uAux[id2]) * invDxPow.x +
                                         (uAux[id3] - 2 * uAux[id] + uAux[id4]) * invDxPow.y);
        }
    }

    for (int i = 0; i < sizeV.x; ++i) {
        for (int j = 0; j < sizeV.y; ++j) {
            const Index2 id(i, j);
            /// Calculate index
            const Index2 id1(clamp(i + 1, 0, (int)sizeV.x - 1), j);
            const Index2 id2(clamp(i - 1, 0, (int)sizeV.x - 1), j);
            const Index2 id3(i, clamp(j + 1, 0, (int)sizeV.y - 1));
            const Index2 id4(i, clamp(j - 1, 0, (int)sizeV.y - 1));
            /// Compute viscosity in V
            velocityY[id] += dtVisDen * ((vAux[id1] - 2 * vAux[id] + vAux[id2]) * invDxPow.x +
                                         (vAux[id3] - 2 * vAux[id] + vAux[id4]) * invDxPow.y);
        }
    }
}

// pressure
void Fluid2::fluidPressureProjection(const float dt)
{
    // pressure
    const Index2 sizeP = pressure.getSize();
    const Index2 sizeU = velocityX.getSize();
    const Index2 sizeV = velocityY.getSize();

    const Vector2 dx(grid.getCellDx());
    const Vector2 invDx(1.0f / dx.x, 1.0f / dx.y);
    const Vector2 invDxPow(1.0f / pow(dx.x, 2), 1.0f / pow(dx.y, 2));

    /// wall boundary conditions
    for (unsigned int j = 0; j < sizeU.y; ++j) {
        /// Left wall as solid
        velocityX[Index2(0, j)] = 0.0f;
        /// Right wall as solid
        velocityX[Index2(sizeU.x - 1, j)] = 0.0f;
    }
    for (unsigned int i = 0; i < sizeV.x; ++i) {
        /// Botton as solid
        velocityY[Index2(i, 0)] = 0.0f;
        /// Top as solid
        velocityY[Index2(i, sizeV.y - 1)] = 0.0f;
    }

    /// b
    const float pDt = Scene::kDensity / dt;
    std::vector<double> b(sizeP.x * sizeP.y);
    for (unsigned int i = 0; i < sizeP.x; ++i) {
        for (unsigned int j = 0; j < sizeP.y; ++j) {
            const Index2 id(i, j);
            b[pressure.getLinearIndex(i, j)] = -pDt * ((velocityX[Index2(i + 1, j)] - velocityX[id]) * invDx.x +
                                                       (velocityY[Index2(i, j + 1)] - velocityY[id]) * invDx.y);
        }
    }

    /// A
    SparseMatrix<double> A(sizeP.x * sizeP.y, 5);
    for (unsigned int i = 0; i < sizeP.x; ++i) {
        for (unsigned int j = 0; j < sizeP.y; ++j) {
            const unsigned int id = pressure.getLinearIndex(i, j);
            if (i > 0) {
                const unsigned int id1 = pressure.getLinearIndex(i - 1, j);
                A.add_to_element(id, id, 1. * invDxPow.x);
                A.add_to_element(id, id1, -1. * invDxPow.x);
            }
            if (i < sizeP.x - 1) {
                const unsigned int id1 = pressure.getLinearIndex(i + 1, j);
                A.add_to_element(id, id, 1. * invDxPow.x);
                A.add_to_element(id, id1, -1. * invDxPow.x);
            }
            if (j > 0) {
                const unsigned int id1 = pressure.getLinearIndex(i, j - 1);
                A.add_to_element(id, id, 1. * invDxPow.y);
                A.add_to_element(id, id1, -1. * invDxPow.y);
            }
            A.add_to_element(id, id, 1. * invDxPow.y);
            /// Top as air
            // if (j < sizeP.y - 1) {
            //	const unsigned int id1 = pressure.getLinearIndex(i, j + 1);
            //	A.add_to_element(id, id1, -1. * invDxPow.y);
            //}
            /// Top as solid
            if (j < sizeP.y - 1) {
                const unsigned int id1 = pressure.getLinearIndex(i, j + 1);
                A.add_to_element(id, id1, -1. * invDxPow.y);
            }
        }
    }

    // pcg solver
    PCGSolver<double> solver;

    double residual_out;
    int iterations_out;
    std::vector<double> p(sizeP.x * sizeP.y);

    solver.set_solver_parameters(1e-6, 10000);
    solver.solve(A, b, p, residual_out, iterations_out);

    std::cout << "Pressure system result: res=" << residual_out << ", iter=" << iterations_out << std::endl;

    // set pressure
    for (unsigned int i = 0; i < sizeP.x; ++i) {
        for (unsigned int j = 0; j < sizeP.y; ++j) {
            const Index2 id(i, j);
            pressure[id] = (float)p[pressure.getLinearIndex(i, j)];
        }
    }

    // apply pressure gradient
    const float dtOverRho = dt / Scene::kDensity;
    for (unsigned int i = 1; i < sizeU.x - 1; ++i) {
        for (unsigned int j = 0; j < sizeU.y; ++j) {
            const Index2 id(i, j);
            const Index2 id1(i - 1, j);
            const float gradp = (pressure[id] - pressure[id1]) * invDx.x;
            velocityX[id] -= dtOverRho * gradp;
        }
    }
    for (unsigned int i = 0; i < sizeV.x; ++i) {
        for (unsigned int j = 1; j < sizeV.y - 1; ++j) {
            const Index2 id(i, j);
            const Index2 id1(i, j - 1);
            const float gradp = (pressure[id] - pressure[id1]) * invDx.y;
            velocityY[id] -= dtOverRho * gradp;
        }
    }

    if (flipEnabled) {
        for (unsigned int i = 0; i < particles.getSize(); ++i) {
            const Vector2 &posParticle = particles.getPosition(i);
            Vector2 oldVel, newVel;
            //oldVel.x = bilerp(grid.getFaceIndex(posParticle, 0), oldVelocityX, oldVelocityX.getSize());
            oldVel.x = bilerp(grid.getFaceIndex(posParticle, 0), oldVelocityX, oldVelocityX.getSize());
            oldVel.y = bilerp(grid.getFaceIndex(posParticle, 1), oldVelocityY, oldVelocityY.getSize());
            newVel.x = bilerp(grid.getFaceIndex(posParticle, 0), velocityX, velocityX.getSize());
            newVel.y = bilerp(grid.getFaceIndex(posParticle, 1), velocityY, velocityY.getSize());
            // calculate FLIP velocity delta
            const Vector2 velDelta = newVel - oldVel;

            // apply PIC/FLIP to update particles velocities
            const Vector2 picFlipVel = 0.95 * (particles.getVelocity(i) + velDelta) + 0.05 * newVel;
            particles.setVelocity(i, picFlipVel);
        }
    }
}
}  // namespace asa
