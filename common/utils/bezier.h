//
// Created by kai on 19-11-3.
//

#ifndef PYTHON2CPP_BEZIER_H
#define PYTHON2CPP_BEZIER_H

#pragma once
#include <assert.h>
#include <math.h>
#include <vector>
#include <limits>
#include <algorithm>    // std::minmax

// Cross platform define for printing size_t variables
#ifdef __WIN32__
#define BEZIER_SIZE_T_FORMAT "%ld"
#else
#define BEZIER_SIZE_T_FORMAT "%zd"
#endif

#define BEZIER_FUZZY_EPSILON 0.0001
#define BEZIER_DEFAULT_INTERVALS 10
#define BEZIER_DEFAULT_MAX_ITERATIONS 15
namespace Bezier
{
    namespace Math
    {
        inline double faculty(std::size_t n)
        {
            double d = 1.0;
            if (n == 0 || n == 1)
                return d;
            for (std::size_t i = 2; i <= n; i++)
                d *= (double) i;
            return d;
        }

        // Note: Using faculty function seems to be way faster than the recursive one
        // given at https://en.wikipedia.org/wiki/Binomial_coefficient
        inline double binomial(std::size_t n, std::size_t k)
        {
            assert(k <= n);
            return faculty(n) / (faculty(k) * faculty(n - k));
        }

        inline bool isWithinZeroAndOne(double x)
        {
            return x >= -BEZIER_FUZZY_EPSILON && x <= (1.0 + BEZIER_FUZZY_EPSILON);
        }
    }

    template<std::size_t N>
    class BinomialCoefficients
    {
    public:
        BinomialCoefficients()
        {
            std::size_t center = N / 2;
            std::size_t k = 0;

            while (k <= center)
            {
                mCoefficients[k] = Math::binomial(N, k);
                k++;
            }

            // Utilize the symmetrical nature of the binomial coefficients.
            while (k <= N)
            {
                mCoefficients[k] = mCoefficients[N - k];
                k++;
            }
        }

        static constexpr std::size_t size()
        {
            return N + 1;
        }

        const std::size_t operator [](std::size_t idx) const
        {
            assert(idx < size());
            return mCoefficients[idx];
        }

    private:
        double mCoefficients[size()];
    };

    struct PolynomialPair
    {
        std::size_t t = 0;
        std::size_t one_minus_t = 0;

        double valueAt(double t) const
        {
            return pow(1 - t, one_minus_t) * pow(t, this->t);
        }
    };

    template<std::size_t N>
    class PolynomialCoefficients
    {
    public:
        PolynomialCoefficients()
        {
            for (std::size_t i = 0; i <= N; i++)
            {
                mPolynomialPairs[i].t = i;
                mPolynomialPairs[i].one_minus_t = N - i;
                assert(mPolynomialPairs[i].t + mPolynomialPairs[i].one_minus_t == N);
            }
        }

        double valueAt(std::size_t pos, double t) const
        {
            assert(pos < size());
            return mPolynomialPairs[pos].valueAt(t);
        }

        static constexpr std::size_t size()
        {
            return N + 1;
        }

        const PolynomialPair& operator [](std::size_t idx) const
        {
            assert(idx < size());
            return mPolynomialPairs[idx];
        }

    private:
        PolynomialPair mPolynomialPairs[size()];
    };

    class Vec2
    {
    public:
        Vec2()
                : x(0)
                , y(0)
        {}

        Vec2(double x, double y)
                : x(x)
                , y(y)
        {}

        Vec2(double x, double y, bool normalize)
                : x(x)
                , y(y)
        {
            if (normalize)
                this->normalize();
        }

        Vec2(const Vec2& other)
                : x(other.x)
                , y(other.y)
        {}

        Vec2(const Vec2& other, bool normalize)
                : Vec2(other.x, other.y, normalize)
        {}

        void set(double x, double y)
        {
            this->x = x;
            this->y = y;
        }

        void set(const Vec2& other)
        {
            this->x = other.x;
            this->y = other.y;
        }

        double length() const
        {
            return sqrt(x*x + y*y);
        }

        void normalize()
        {
            double len = length();
            x /= len;
            y /= len;
        }

        void translate(double dx, double dy)
        {
            x += dx;
            y += dy;
        }

        void translate(const Vec2& distance)
        {
            x += distance.x;
            y += distance.y;
        }

        void rotate(double angle, const Vec2& pivot = Vec2(0, 0))
        {
            double s = sin(angle);
            double c = cos(angle);

            x -= pivot.x;
            y -= pivot.y;

            double xnew = x * c - y * s;
            double ynew = x * s + y * c;

            x = xnew + pivot.x;
            y = ynew + pivot.y;
        }

        double angle() const
        {
            return atan2(y, x);
        }

        double angleDeg() const
        {
            return angle() * 180.0 / M_PI;
        }

        double operator[](std::size_t axis) const
        {
            assert(axis < Vec2::size);
            switch (axis)
            {
                case 0:
                    return x;
                case 1:
                    return y;
                default:
                    //fprintf(stderr, "Erroneous axis: " BEZIER_SIZE_T_FORMAT ". Returning 0.\n", axis);
                    return 0;
            }
        }

        double& operator[](std::size_t axis)
        {
            assert(axis < Vec2::size);
            switch (axis)
            {
                case 0:
                    return x;
                case 1:
                    return y;
                default:
                    //fprintf(stderr, "Erroneous axis: " BEZIER_SIZE_T_FORMAT ". Returning x.\n", axis);
                    return x;
            }
        }

        Vec2 operator+(const Vec2& other) const
        {
            return Vec2(x + other.x, y + other.y);
        }

        Vec2 operator-(const Vec2& other) const
        {
            return Vec2(x - other.x, y - other.y);
        }

        Vec2 operator-() const
        {
            return Vec2(-x, -y);
        }

        Vec2 operator*(double scale) const
        {
            return Vec2(x * scale, y * scale);
        }

        Vec2 operator/(double scale) const
        {
            return Vec2(x / scale, y / scale);
        }

        Vec2 operator/(const Vec2& other) const
        {
            return Vec2(x / other.x, y / other.y);
        }

        bool fuzzyEquals(const Vec2& other) const
        {
            bool equals = true;
            for (std::size_t axis = 0; axis < Vec2::size; axis++)
            {
                if (fabs((*this)[axis] - other[axis]) >= BEZIER_FUZZY_EPSILON)
                {
                    equals = false;
                    break;
                }
            }
            return equals;
        }

        bool isWithinZeroAndOne() const
        {
            return Math::isWithinZeroAndOne(x) && Math::isWithinZeroAndOne(y);
        }

        double x;
        double y;
        static constexpr std::size_t size = 2;
    };

    typedef Vec2 Point;
    typedef Vec2 Normal;
    typedef Vec2 Tangent;

    struct ExtremeValue
    {
        ExtremeValue(double t, std::size_t axis)
                : t(t)
                , axis(axis)
        {}

        bool fuzzyEquals(const ExtremeValue& other) const
        {
            return axis == other.axis && fabs(t - other.t) < BEZIER_FUZZY_EPSILON;
        }

        const double t;
        const std::size_t axis;
    };

    class ExtremeValues
    {
    public:
        bool add(double t, std::size_t axis)
        {
            return add(ExtremeValue(t, axis));
        }

        bool add(const ExtremeValue& val)
        {
            assert(Math::isWithinZeroAndOne(val.t));
            for (auto const &v : values)
            {
                if (val.fuzzyEquals(v))
                    return false;
            }
            values.push_back(val);
            return true;
        }

        std::size_t size() const
        {
            return values.size();
        }

        ExtremeValue& operator[](std::size_t idx)
        {
            assert(idx < values.size());
            return values[idx];
        }

        ExtremeValue operator[](std::size_t idx) const
        {
            assert(idx < values.size());
            return values[idx];
        }

    private:
        std::vector<ExtremeValue> values;
    };

    class ExtremePoints
    {
    public:
        bool add(double x, double y)
        {
            return add(Point(x, y));
        }

        bool add(const Point& extremePoint)
        {
            for (auto const &ep : points)
            {
                if (extremePoint.fuzzyEquals(ep))
                    return false;
            }
            points.push_back(extremePoint);
            return true;
        }

        std::size_t size() const
        {
            return points.size();
        }

        bool empty() const
        {
            return !size();
        }

        Point& operator[](std::size_t idx)
        {
            assert(idx < size());
            return points[idx];
        }

        Point operator[](std::size_t idx) const
        {
            assert(idx < size());
            return points[idx];
        }

    private:
        std::vector<Point> points;
    };

    class AxisAlignedBoundingBox
    {
    public:
        AxisAlignedBoundingBox(const Point& p0, const Point& p1, const Point& p2, const Point& p3)
                : points{{p0}, {p1}, {p2}, {p3}}
        {}

        AxisAlignedBoundingBox(const ExtremePoints& xPoints)
        {
            double minX = std::numeric_limits<double>::max();
            double maxX = -std::numeric_limits<double>::max();
            double minY = std::numeric_limits<double>::max();
            double maxY = -std::numeric_limits<double>::max();

            for (std::size_t i = 0; i < xPoints.size(); i++)
            {
                if (xPoints[i].x > maxX)
                    maxX = xPoints[i].x;
                if (xPoints[i].x < minX)
                    minX = xPoints[i].x;
                if (xPoints[i].y > maxY)
                    maxY = xPoints[i].y;
                if (xPoints[i].y < minY)
                    minY = xPoints[i].y;
            }

            points[0].set(minX, minY);
            points[1].set(minX, maxY);
            points[2].set(maxX, maxY);
            points[3].set(maxX, minY);
        }

        static constexpr std::size_t size()
        {
            return 4;
        }

        double minX() const
        {
            return points[0].x;
        }

        double maxX() const
        {
            return points[2].x;
        }

        double minY() const
        {
            return points[0].y;
        }

        double maxY() const
        {
            return points[2].y;
        }

        double width() const
        {
            return maxX() - minX();
        }

        double height() const
        {
            return maxY() - minY();
        }

        double area() const
        {
            return ((double) width() * (double) height());
        }

        Point& operator[](std::size_t idx)
        {
            assert(idx < size());
            return points[idx];
        }

        Point operator[](std::size_t idx) const
        {
            assert(idx < size());
            return points[idx];
        }

    private:
        Point points[4]; // Starting in lower left corner, going clock-wise.
    };

    typedef AxisAlignedBoundingBox AABB;

    class TightBoundingBox
    {
    public:
        // Takes the ExtremePoints of the Bezier curve moved to origo and rotated to align the x-axis
        // as arguments as well as the translation/rotation used to calculate it.
        TightBoundingBox(const ExtremePoints& xPoints, const Vec2& translation, double rotation)
        {
            double minX = std::numeric_limits<double>::max();
            double maxX = -std::numeric_limits<double>::max();
            double minY = std::numeric_limits<double>::max();
            double maxY = -std::numeric_limits<double>::max();

            for (std::size_t i = 0; i < xPoints.size(); i++)
            {
                if (xPoints[i].x > maxX)
                    maxX = xPoints[i].x;
                if (xPoints[i].x < minX)
                    minX = xPoints[i].x;
                if (xPoints[i].y > maxY)
                    maxY = xPoints[i].y;
                if (xPoints[i].y < minY)
                    minY = xPoints[i].y;
            }

            points[0].set(minX, minY);
            points[1].set(minX, maxY);
            points[2].set(maxX, maxY);
            points[3].set(maxX, minY);

            if (xPoints.empty())
                return;

            for (std::size_t i = 0; i < 4; i++)
            {
                points[i].rotate(-rotation);
                points[i].translate(-translation);
            }
        }

        static constexpr std::size_t size()
        {
            return 4;
        }

        double minX() const
        {
            auto result = std::minmax({points[0].x, points[1].x, points[2].x, points[3].x});
            return result.first;
        }

        double maxX() const
        {
            auto result = std::minmax({points[0].x, points[1].x, points[2].x, points[3].x});
            return result.second;
        }

        double minY() const
        {
            auto result = std::minmax({points[0].y, points[1].y, points[2].y, points[3].y});
            return result.first;
        }

        double maxY() const
        {
            auto result = std::minmax({points[0].y, points[1].y, points[2].y, points[3].y});
            return result.second;
        }

        double area() const
        {
            return width() * height();
        }

        // Uses the two first points to calculate the "width".
        double width() const
        {
            double x = points[1].x - points[0].x;
            double y = points[1].y - points[0].y;
            return sqrt(x * x + y * y);
        }

        // Uses the second and third points to calculate the "height".
        double height() const
        {
            double x = points[2].x - points[1].x;
            double y = points[2].y - points[1].y;
            return sqrt(x * x + y * y);
        }

        Point& operator[](std::size_t idx)
        {
            assert(idx < size());
            return points[idx];
        }

        Point operator[](std::size_t idx) const
        {
            assert(idx < size());
            return points[idx];
        }

    private:
        Point points[4]; // The points are ordered in a clockwise manner.
    };

    typedef TightBoundingBox TBB;

    template <std::size_t N>
    class Bezier
    {
    public:
        Bezier()
        {
            for (std::size_t i = 0; i < N+1; i++)
                mControlPoints[i].set(0, 0);
        }

        Bezier(const std::vector<Point>& controlPoints)
        {
            assert(controlPoints.size() == size()); // The Bezier curve must be initialized with the expected number og points
            for (std::size_t i = 0; i < controlPoints.size(); i++)
                mControlPoints[i] = Point(controlPoints[i]);
        }

        Bezier(const Bezier<N>& other)
        {
            for (std::size_t i = 0; i < other.size(); i++)
                mControlPoints[i] = Point(other[i]);
        }

        // The order of the bezier curve.
        std::size_t order() const
        {
            return N;
        }

        // Number of control points.
        std::size_t size() const
        {
            return N + 1;
        }

        Bezier<N-1> derivative() const
        {
            assert(N != 0);

            // Note: derivative weights/control points are not actual control points.
            std::vector<Point> derivativeWeights(N);
            for (std::size_t i = 0; i < N; i++)
                derivativeWeights[i].set(Point((mControlPoints[i+1] - mControlPoints[i]) * N));

            return Bezier<N-1>(derivativeWeights);
        }

    public:
        double valueAt(double t, std::size_t axis) const
        {
            assert(axis < Vec2::size); // Currently only support 2D
            double sum = 0;
            for (std::size_t n = 0; n < N+1; n++)
            {
                sum += binomialCoefficients[n] * polynomialCoefficients[n].valueAt(t) * mControlPoints[n][axis];
            }
            return sum;
        }

        Point valueAt(double t) const
        {
            Point p;
            for (std::size_t i = 0; i < Point::size; i++)
            {
                p[i] = (double) valueAt(t, i);
            }
            return p;
        }

        Tangent tangentAt(double t, bool normalize = true) const
        {
            Point p;
            Bezier<N-1> derivative = this->derivative();
            p.set(derivative.valueAt(t));
            if (normalize)
                p.normalize();
            return p;
        }

        Normal normalAt(double t, bool normalize = true) const
        {
            Point tangent = tangentAt(t, normalize);
            return Normal(-tangent.y, tangent.x, normalize);
        }

        void translate(const Vec2& distance)
        {
            for (std::size_t i = 0; i < N+1; i++)
            {
                mControlPoints[i].translate(distance);
            }
        }

        void translate(double dx, double dy)
        {
            for (std::size_t i = 0; i < N+1; i++)
            {
                mControlPoints[i].translate(dx, dy);
            }
        }

        void rotate(double angle, Vec2 pivot = Vec2(0, 0))
        {
            for (std::size_t i = 0; i < N+1; i++)
            {
                mControlPoints[i].rotate(angle, pivot);
            }
        }

        ExtremeValues derivativeZero(std::size_t intervals = BEZIER_DEFAULT_INTERVALS,
                                     double epsilon = BEZIER_FUZZY_EPSILON,
                                     std::size_t maxIterations = BEZIER_DEFAULT_MAX_ITERATIONS) const
        {
            switch (N)
            {
                case 1:
                    return derivativeZero1();
                case 2:
                    return derivativeZero2();
                case 3:
//                    return derivativeZero3();
                    return newtonRhapson(intervals, epsilon, maxIterations);
                default:
                    return newtonRhapson(intervals, epsilon, maxIterations);
            }
        }

        ExtremePoints extremePoints() const
        {
            ExtremeValues xVals = derivativeZero();
            xVals.add(0.0f, 0);
            xVals.add(1.0f, 0);

            ExtremePoints xPoints;
            for (std::size_t i = 0; i < xVals.size(); i++)
                xPoints.add(valueAt(xVals[i].t));

            return xPoints;
        }

        AxisAlignedBoundingBox aabb() const
        {
            return AxisAlignedBoundingBox(extremePoints());
        }

        AxisAlignedBoundingBox aabb(const ExtremePoints& xPoints) const
        {
            return AxisAlignedBoundingBox(xPoints);
        }

        TightBoundingBox tbb() const
        {
            Bezier<N> bezier = *this;

            // Translate last control point (highest order) to origo.
            Vec2 translation(-bezier[N]);
            bezier.translate(translation);

            // Rotate bezier to align the first control point (lowest order) with the x-axis
            double angle = -bezier[0].angle();
            bezier.rotate(angle);

            return TightBoundingBox(bezier.extremePoints(), translation, angle);
        }

    public:
        Point& operator [](std::size_t idx)
        {
            assert(idx < size());
            return mControlPoints[idx];
        }

        Point operator [](std::size_t idx) const
        {
            assert(idx < size());
            return mControlPoints[idx];
        }

    private:
        ExtremeValues derivativeZero1() const
        {
            assert(N == 1);
            return ExtremeValues();
        }

        ExtremeValues derivativeZero2() const
        {
            assert(N == 2);
            ExtremeValues xVals;
            Point roots = (mControlPoints[0] - mControlPoints[1]) / (mControlPoints[0] - mControlPoints[1] * 2 + mControlPoints[2]);
            if (Math::isWithinZeroAndOne(roots[0]))
                xVals.add(roots[0], 0);
            if (Math::isWithinZeroAndOne(roots[1]))
                xVals.add(roots[1], 1);
            return xVals;
        }

        ExtremeValues derivativeZero3() const
        {
            // Note: NOT IMPLMENTED YET
            assert(N == 3);
            return ExtremeValues();
        }

        ExtremeValues newtonRhapson(std::size_t intervals = BEZIER_DEFAULT_INTERVALS,
                                    double epsilon = BEZIER_FUZZY_EPSILON,
                                    std::size_t maxIterations = BEZIER_DEFAULT_MAX_ITERATIONS) const
        {
            assert(N >= 2);
            ExtremeValues xVals;
            const double dt = 1.0 / (double) intervals;
            const double absEpsilon = fabs(epsilon);
            const Bezier<N-1> db = derivative();
            const Bezier<N-2> ddb = db.derivative();

            for (std::size_t i = 0; i < Point::size; i++)
            {
                double t = 0;

                while(t <= 1.0)
                {
                    double zeroVal = t;
                    std::size_t current_iter = 0;

                    while (current_iter < maxIterations)
                    {
                        double dbVal = db.valueAt(zeroVal, i);
                        double ddbVal = ddb.valueAt(zeroVal, i);
                        double nextZeroVal = zeroVal - (dbVal / ddbVal);

                        if (fabs(nextZeroVal - zeroVal) < absEpsilon)
                        {
                            if (Math::isWithinZeroAndOne(nextZeroVal))
                            {
                                xVals.add(nextZeroVal, i);
                                break;
                            }
                        }

                        zeroVal = nextZeroVal;
                        current_iter++;
                    }

                    t += dt;
                }
            }

            return xVals;
        }

    public:
        static const BinomialCoefficients<N> binomialCoefficients;
        static const PolynomialCoefficients<N> polynomialCoefficients;

    private:
        Point mControlPoints[N+1];
    };

    template<std::size_t N>
    const BinomialCoefficients<N> Bezier<N>::binomialCoefficients = BinomialCoefficients<N>();

    template<std::size_t N>
    const PolynomialCoefficients<N> Bezier<N>::polynomialCoefficients = PolynomialCoefficients<N>();

} // namespace Bezier

#endif //PYTHON2CPP_BEZIER_H
