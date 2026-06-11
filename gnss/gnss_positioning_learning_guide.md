# GNSS Positioning — A Complete Learning Guide
### From Math Basics to libgnss++ Source Code

> **Who this is for:** A developer who can write basic C++, has high-school-level math (algebra, maybe a little geometry), and zero prior GNSS knowledge. By the end you will be able to read every formula in libgnss++ and understand *why* it is there.

> **Not sure your math is strong enough?** Start at [Part −1](#part--1--prerequisite-math-map-absolute-basics). It covers the bare-minimum building blocks — arithmetic rules, exponents, fractions, and functions — before moving into the GNSS-specific math in Part 0.

---

## Table of Contents

**Part −1 — Prerequisite Math Map (Absolute Basics)**
- [−1.1 Numbers and the Number Line](#-11-numbers-and-the-number-line)
- [−1.2 Arithmetic: The Four Operations and Their Rules](#-12-arithmetic-the-four-operations-and-their-rules)
- [−1.3 Fractions, Ratios, and Percentages](#-13-fractions-ratios-and-percentages)
- [−1.4 Powers and Roots](#-14-powers-and-roots)
- [−1.5 Variables and Basic Algebra](#-15-variables-and-basic-algebra)
- [−1.6 Functions and Graphs](#-16-functions-and-graphs)
- [−1.7 The Coordinate Plane (2D)](#-17-the-coordinate-plane-2d)

**Part 0 — Math Foundations (Start Here)**
- [0.1 How to Read Math Notation](#01-how-to-read-math-notation)
- [0.2 Trigonometry](#02-trigonometry)
- [0.3 3D Space and Distance](#03-3d-space-and-distance)
- [0.4 Vectors](#04-vectors)
- [0.5 Matrices](#05-matrices)
- [0.6 Least Squares — Solving Many Equations at Once](#06-least-squares--solving-many-equations-at-once)
- [0.7 Statistics — Noise, Error, and Uncertainty](#07-statistics--noise-error-and-uncertainty)
- [0.8 Iterative Methods](#08-iterative-methods)
- [0.9 Linearization — Turning Curves into Lines](#09-linearization--turning-curves-into-lines)
- [0.10 Logarithms and the dB Scale](#010-logarithms-and-the-db-scale)

**Part 1 — GNSS Positioning**
1. [What is GNSS? (Big Picture)](#1-what-is-gnss-big-picture)
2. [The World of Signals and Frequencies](#2-the-world-of-signals-and-frequencies)
3. [How We Describe Position on Earth](#3-how-we-describe-position-on-earth)
4. [GNSS Time Systems](#4-gnss-time-systems)
5. [What the Receiver Measures (Observations)](#5-what-the-receiver-measures-observations)
6. [Where Are the Satellites? (Ephemeris)](#6-where-are-the-satellites-ephemeris)
7. [Error Sources: What Corrupts Our Measurements](#7-error-sources-what-corrupts-our-measurements)
8. [SPP — Single Point Positioning](#8-spp--single-point-positioning)
9. [RTK — Real-Time Kinematic](#9-rtk--real-time-kinematic)
10. [PPP — Precise Point Positioning](#10-ppp--precise-point-positioning)
11. [The Kalman Filter (State Estimation)](#11-the-kalman-filter-state-estimation)
12. [Integer Ambiguity Resolution (LAMBDA)](#12-integer-ambiguity-resolution-lambda)
13. [Quality Indicators and Validation](#13-quality-indicators-and-validation)
14. [Data Formats](#14-data-formats)
15. [libgnss++ Architecture Walkthrough](#15-libgnss-architecture-walkthrough)
16. [Summary: The Three Positioning Modes](#16-summary-the-three-positioning-modes)

---

# Part −1 — Prerequisite Math Map (Absolute Basics)

> This section is for readers who feel shaky on basic math. If you can already manipulate simple algebra equations and understand what $x^2$ or $\sqrt{x}$ means, you can skip straight to [Part 0](#part-0--math-foundations).
>
> Every concept here is used directly in Part 0 or Part 1. Nothing is included just for completeness.

---

## −1.1 Numbers and the Number Line

### Types of numbers

Math uses different kinds of numbers, and GNSS code works with all of them:

| Type        | Examples              | Where you see it in GNSS                                 |
|-------------|-----------------------|----------------------------------------------------------|
| Integer     | …, −2, −1, 0, 1, 2, … | GPS week number, satellite PRN, ambiguity integer $N$    |
| Decimal     | 3.14, −0.0069, 1575.42 | Frequency in MHz, lat/lon in degrees, range in meters   |
| Very large  | 6,378,137             | Earth's radius in meters                                 |
| Very small  | 0.0000000729          | Earth's rotation rate in rad/s                           |

### Scientific notation

Writing very large or very small numbers is tedious. Scientific notation compacts them:

$$6{,}378{,}137 \text{ m} = 6.378137 \times 10^6 \text{ m}$$
$$0.000\ 000\ 0729\ \text{rad/s} = 7.29 \times 10^{-8}\ \text{rad/s}$$

In C++ this is written `6.378137e6` and `7.29e-8`. You will see this constantly in `constants.hpp`:
```cpp
constexpr double GPS_L1_FREQ = 1575.42e6;     // 1,575,420,000 Hz
constexpr double OMEGA_E     = 7.2921151467e-5; // 0.0000729... rad/s
```

### The number line

All real numbers sit on an infinitely long line. Positive numbers are to the right of zero, negative to the left. Distance from zero is the **absolute value** $|x|$:

$$|-5| = 5, \quad |3.7| = 3.7, \quad |0| = 0$$

In libgnss++ you often see `std::abs(t)` to check if a time difference is within some tolerance, regardless of direction.

---

## −1.2 Arithmetic: The Four Operations and Their Rules

### Operations and their names

| Operation      | Symbol  | Result name | Example              |
|----------------|---------|-------------|----------------------|
| Addition       | $a + b$ | sum         | $3 + 4 = 7$          |
| Subtraction    | $a - b$ | difference  | $10 - 6 = 4$         |
| Multiplication | $a \times b$ or $a \cdot b$ or $ab$ | product | $3 \times 4 = 12$ |
| Division       | $a / b$ or $\frac{a}{b}$ | quotient | $12 / 4 = 3$ |

### Order of operations (PEMDAS)

When an expression has mixed operations, evaluate in this order:
1. **P**arentheses first: $(3 + 4) \times 2 = 14$, not $3 + (4 \times 2) = 11$
2. **E**xponents: $2^3 = 8$ before multiplying
3. **M**ultiplication and **D**ivision left-to-right
4. **A**ddition and **S**ubtraction left-to-right

This is not just a school rule — it is how C++ operators work too. Parentheses in code exist for the same reason.

### Distributive law

$$a(b + c) = ab + ac$$

Example: $3(x + 5) = 3x + 15$

This is used when expanding matrix-vector products and when rearranging observation equations in GNSS.

### Negative × negative = positive

$$(-1)\times(-1) = +1$$

This appears when computing residuals: a satellite that is farther than predicted gives a *positive* residual, one that is closer gives a *negative* residual. Their squares are always positive, which is why least squares minimizes the **sum of squares** (all non-negative).

---

## −1.3 Fractions, Ratios, and Percentages

### Fractions

A fraction $\frac{a}{b}$ means "$a$ divided by $b$". Rules:

$$\frac{a}{b} \times \frac{c}{d} = \frac{ac}{bd} \qquad \frac{a}{b} \div \frac{c}{d} = \frac{a}{b} \times \frac{d}{c} = \frac{ad}{bc}$$

In GNSS, the wavelength formula is a division:
$$\lambda = \frac{c}{f} = \frac{299{,}792{,}458}{1{,}575{,}420{,}000} \approx 0.1903 \text{ m}$$

### Ratios

A ratio compares two quantities. The ionosphere-free combination coefficients are pure ratios of squared frequencies:

$$C_1 = \frac{f_2^2}{f_2^2 - f_1^2}$$

The numbers cancel their units, leaving a dimensionless multiplier.

### Unit conversions are fractions

Converting 15 degrees to radians:
$$15° \times \frac{\pi\text{ rad}}{180°} = \frac{15\pi}{180} \approx 0.2618\text{ rad}$$

The degree units cancel in the fraction. Always ask: "what do I multiply by to cancel the old unit and introduce the new one?"

---

## −1.4 Powers and Roots

### Integer powers

$a^n$ means multiply $a$ by itself $n$ times:
$$2^3 = 2 \times 2 \times 2 = 8, \qquad 10^6 = 1{,}000{,}000$$

Special cases:
$$a^0 = 1, \qquad a^1 = a, \qquad a^{-1} = \frac{1}{a}, \qquad a^{-n} = \frac{1}{a^n}$$

In GNSS, ionospheric delay scales as $1/f^2 = f^{-2}$. A higher frequency gets less ionospheric delay.

### Power rules (crucial for simplifying formulas)

$$a^m \times a^n = a^{m+n}$$
$$\frac{a^m}{a^n} = a^{m-n}$$
$$(a^m)^n = a^{mn}$$
$$(ab)^n = a^n b^n$$

Example used in GNSS: the Earth gravitational constant appears as $\mu / a^3$ under a square root — this is $\mu^{1/2} \cdot a^{-3/2}$.

### Square roots

The square root $\sqrt{x}$ is the number that, when squared, gives $x$:
$$\sqrt{9} = 3 \quad \text{because} \quad 3^2 = 9$$
$$\sqrt{x} = x^{1/2}$$

The GPS navigation message broadcasts $\sqrt{a}$ (not $a$ directly). To get the semi-major axis:
```cpp
const double a = eph.sqrt_a * eph.sqrt_a;  // a = (√a)²
```

The Euclidean distance formula is a square root:
$$d = \sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2}$$

In Eigen: `(sat_pos - rec_pos).norm()` computes exactly this.

### Fractional exponents

In general $a^{p/q} = \sqrt[q]{a^p}$. The most common:
$$a^{1/2} = \sqrt{a}, \qquad a^{3/2} = a \cdot \sqrt{a}, \qquad a^{-1/2} = \frac{1}{\sqrt{a}}$$

---

## −1.5 Variables and Basic Algebra

### Variables

A variable is a letter that stands for an unknown or changing number. When you write $\rho = c \cdot \tau$, you are saying: "geometric range equals speed of light times travel time". Any specific numbers could be plugged in.

### Solving a linear equation

**Goal:** isolate the variable on one side.

Strategy: whatever you do to one side, do the same to the other.

$$3x + 5 = 20$$
$$3x = 15 \quad \text{(subtract 5 from both sides)}$$
$$x = 5 \quad \text{(divide both sides by 3)}$$

This is exactly what happens in SPP: the position equation is rearranged so that unknown position corrections are on the left and known measurement residuals are on the right.

### Simultaneous equations

Two unknowns require two equations. Four unknowns (x, y, z, clock) require four equations — hence the minimum of 4 satellites in SPP.

$$\begin{cases} x + y = 10 \\ x - y = 4 \end{cases}$$

Add the two equations: $2x = 14 \Rightarrow x = 7$. Substitute: $y = 3$.

With 12 satellites and 4 unknowns, we have more equations than unknowns → least squares (Section 0.6).

### Substitution and plugging in

If you know $f = c / \lambda$, and $c = 299792458$ and $f = 1575.42 \times 10^6$, then:
$$\lambda = \frac{c}{f} = \frac{299792458}{1575.42 \times 10^6} \approx 0.1903 \text{ m}$$

Most of the GNSS formulas are just this: substitute known values, solve for the unknown.

---

## −1.6 Functions and Graphs

### What is a function?

A function is a machine: you put in one number, you get out exactly one number.

$$f(x) = 2x + 3 \qquad \Rightarrow \qquad f(5) = 13, \quad f(-1) = 1$$

In GNSS, the pseudorange is a function of receiver position: $P = f(x, y, z, t)$. When the receiver moves, the pseudorange changes in a predictable way.

### Linear functions

$$f(x) = mx + b$$

- $m$ = slope (how steep the line is)
- $b$ = y-intercept (value when $x = 0$)

The satellite clock correction is a linear (actually quadratic) function of time:
$$\delta t^s = a_{f0} + a_{f1}(t - t_{oc}) + a_{f2}(t - t_{oc})^2$$

$a_{f0}$ is the bias (intercept), $a_{f1}$ is the drift (slope), $a_{f2}$ is the drift rate (curvature).

### Quadratic functions and parabolas

$$f(x) = ax^2 + bx + c$$

The graph is a U-shape (parabola). The squared terms in the satellite clock model ($a_{f2} \cdot t^2$) and in least-squares minimization ($\sum e_i^2$) are quadratic.

### Why graphs help intuition

Even if you never plot anything, imagining a graph helps:
- A **steep line** means small changes in $x$ cause large changes in $f$ → the variable has high influence.
- A **flat line** means almost no influence. In DOP calculations, a satellite directly overhead adds almost no information to the horizontal position.
- A **curve** means the linear approximation (Section 0.9) is only valid nearby.

---

## −1.7 The Coordinate Plane (2D)

### The Cartesian plane

Two perpendicular number lines — the x-axis (horizontal) and y-axis (vertical) — divide the plane into four quadrants. Every point is described by $(x, y)$.

```
      y
      |
  Q2  |  Q1
------+------→ x
  Q3  |  Q4
      |
```

### Distance between two 2D points

$$d = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}$$

This is the 2D version of the 3D ECEF range formula. Understanding this 2D case first makes the 3D version obvious.

### Angle of a point from the origin

$$\theta = \arctan\!\left(\frac{y}{x}\right)$$

In practice `atan2(y, x)` is used (handles all four quadrants). This is how longitude is computed from ECEF coordinates:
```cpp
lon = std::atan2(ecef(1), ecef(0));  // lon = atan2(y, x)
```

### Extension to 3D

Adding a third axis $z$ pointing out of the plane gives 3D Cartesian coordinates. The distance formula gains a third term. All ECEF computations in libgnss++ live in this 3D space.

---

# Part 0 — Math Foundations

> You only need to be comfortable with the ideas here before tackling Part 1. None of it is harder than what appears in a high-school physics textbook — but it is presented here with the GNSS context in mind so you see exactly *why* each piece matters.

---

## 0.1 How to Read Math Notation

Before anything else, here is a translation table for symbols that appear constantly in GNSS literature and in the libgnss++ source.

### Greek letters used in this guide

| Symbol | Name    | What it usually means in GNSS                       |
|--------|---------|------------------------------------------------------|
| λ      | lambda  | Wavelength of a radio signal (meters)                |
| φ      | phi     | Latitude, or carrier phase measurement (cycles)      |
| θ / α  | theta / alpha | Angle (radians or degrees)                  |
| σ      | sigma   | Standard deviation (spread / noise)                  |
| σ²     | sigma-squared | Variance (squared spread)                    |
| Δ      | delta   | "Difference" (Δx = change in x)                      |
| δ      | delta (small) | Small correction or small difference          |
| ε      | epsilon | Residual / measurement noise                         |
| τ      | tau     | Time delay or travel time                            |
| ω      | omega   | Angular rate (radians per second)                    |
| Ω      | Omega (capital) | Right ascension / longitude of ascending node |

### Abbreviations and function notation

| Notation     | Meaning                                                      |
|--------------|--------------------------------------------------------------|
| `‖v‖`        | The length (norm) of vector **v**                            |
| `v·w`        | Dot product of two vectors                                   |
| `aᵀ`         | Transpose of matrix or vector **a**                          |
| `A⁻¹`        | Inverse of matrix A                                          |
| `f(x)`       | A function: feed in x, get out f(x)                          |
| `∂f/∂x`      | Partial derivative: how much f changes when only x changes   |
| `≈`          | Approximately equal                                          |
| `∑`          | Sum (add up a list of values)                                |

**Key rule:** In this document, **bold** symbols denote vectors and matrices, while italic *x* denotes a scalar (a single number).

---

## 0.2 Trigonometry

GNSS constantly uses angles and triangles: elevation angle of a satellite, azimuth direction, orbital inclination. You need three functions.

### The three basic functions

Draw a right triangle. Let *θ* be one of the non-right angles, *opp* = the side opposite to it, *adj* = the side adjacent, *hyp* = the hypotenuse.

$$\sin(\theta) = \frac{\text{opp}}{\text{hyp}}, \qquad \cos(\theta) = \frac{\text{adj}}{\text{hyp}}, \qquad \tan(\theta) = \frac{\text{opp}}{\text{adj}}$$

Their inverses let you recover the angle from a ratio:

$$\theta = \arctan\!\left(\frac{y}{x}\right)$$

`atan2(y, x)` is the two-argument version used in libgnss++ — it handles all four quadrants correctly.

### Radians vs degrees

Computers (and GNSS math) almost always use **radians**.

$$1 \text{ full circle} = 360° = 2\pi \text{ rad}$$
$$\text{degrees} \times \frac{\pi}{180} = \text{radians}$$

In libgnss++, the elevation mask is stored in radians:
```cpp
// rtk.hpp
double elevation_mask = 15.0 * M_PI / 180.0;  // 15 degrees in radians
```

### Useful identities

$$\sin^2\theta + \cos^2\theta = 1 \qquad \text{(Pythagorean identity)}$$

This appears in ECEF ↔ geodetic coordinate conversions.

---

## 0.3 3D Space and Distance

Every satellite and receiver lives in 3D space. We label points with three numbers: $(x, y, z)$.

### Euclidean distance

The distance between two points $A = (x_1, y_1, z_1)$ and $B = (x_2, y_2, z_2)$ is:

$$d = \sqrt{(x_2-x_1)^2 + (y_2-y_1)^2 + (z_2-z_1)^2}$$

This is the geometric range between a satellite and a receiver — the foundation of all GNSS positioning.

In libgnss++ (`coordinates.hpp`):
```cpp
double r = (sat_pos - rec_pos).norm();  // Eigen's .norm() does exactly this
```

### Why 3D and not latitude/longitude?

Latitude and longitude are convenient for humans but awkward for math (they are curved). Inside the GNSS engine everything is in **ECEF** (Earth-Centred Earth-Fixed) 3D Cartesian coordinates. Conversion happens only at input and output. More on this in Section 3.

---

## 0.4 Vectors

A vector is a list of numbers that represents a direction and magnitude in space.

### Notation and basic operations

A 3D vector: $\mathbf{v} = \begin{pmatrix} v_x \\ v_y \\ v_z \end{pmatrix}$

**Addition:** $\mathbf{a} + \mathbf{b} = \begin{pmatrix} a_x+b_x \\ a_y+b_y \\ a_z+b_z \end{pmatrix}$

**Scalar multiplication:** $3\mathbf{v} = \begin{pmatrix} 3v_x \\ 3v_y \\ 3v_z \end{pmatrix}$

**Length (norm):** $\|\mathbf{v}\| = \sqrt{v_x^2 + v_y^2 + v_z^2}$

**Unit vector (normalized):** $\hat{\mathbf{v}} = \frac{\mathbf{v}}{\|\mathbf{v}\|}$ — points in the same direction but has length 1.

### Dot product

$$\mathbf{a} \cdot \mathbf{b} = a_x b_x + a_y b_y + a_z b_z = \|\mathbf{a}\|\|\mathbf{b}\|\cos\theta$$

The dot product equals zero when two vectors are perpendicular.

### Why vectors matter in GNSS

The **line-of-sight (LOS) unit vector** from receiver to satellite is used constantly. In the SPP solver (`spp.cpp`):

```cpp
const Vector3d los = (satellite_position - receiver_position).normalized();
```

This unit vector tells us: "if the receiver moves one meter in the x-direction, how much does the measured range change?" The answer is `−los.x`. This builds the geometry matrix **H** that connects position unknowns to range measurements.

---

## 0.5 Matrices

A matrix is a rectangular grid of numbers. Matrices let us compactly write *many* equations at once — which is exactly what we need when we have dozens of satellites and four unknown coordinates.

### Notation

An $m \times n$ matrix **A** has $m$ rows and $n$ columns:

$$\mathbf{A} = \begin{pmatrix} a_{11} & a_{12} & \cdots & a_{1n} \\ a_{21} & a_{22} & \cdots & a_{2n} \\ \vdots & & \ddots & \vdots \\ a_{m1} & a_{m2} & \cdots & a_{mn} \end{pmatrix}$$

### Matrix–vector multiplication

$\mathbf{A}\mathbf{x} = \mathbf{b}$ where **A** is $m \times n$, **x** is $n \times 1$, **b** is $m \times 1$:

$$b_i = \sum_{j=1}^{n} a_{ij} x_j$$

This is exactly how we write "each of the $m$ measured ranges is some combination of the $n$ unknown state components".

### Transpose

$\mathbf{A}^\top$ flips rows and columns: element $(i,j)$ becomes element $(j,i)$.

### Identity matrix

$\mathbf{I}$ has 1s on the diagonal and 0s elsewhere. $\mathbf{I}\mathbf{x} = \mathbf{x}$ — it does nothing.

### Matrix inverse

$\mathbf{A}^{-1}$ is only defined for square matrices and satisfies $\mathbf{A}\mathbf{A}^{-1} = \mathbf{I}$.  
If $\mathbf{A}\mathbf{x} = \mathbf{b}$, then $\mathbf{x} = \mathbf{A}^{-1}\mathbf{b}$.

Not every matrix has an inverse. A matrix that does not is called **singular**. libgnss++ checks for this:
```cpp
// spp.cpp — reject if the geometry matrix cannot be inverted
if (qr.rank() < num_unknowns) {
    solution.status = SolutionStatus::NONE;
    return solution;
}
```

### Covariance matrix

A covariance matrix $\mathbf{P}$ is square, symmetric ($\mathbf{P} = \mathbf{P}^\top$), and its diagonal elements $P_{ii}$ are the variances (squared uncertainties) of each component. Off-diagonal $P_{ij}$ tells you whether component $i$ and $j$ tend to err together.

The solution in libgnss++ always carries a covariance matrix so you know *how uncertain* the position is:
```cpp
struct PositionSolution {
    Matrix3d position_covariance;  // 3x3 covariance of [x,y,z]
    ...
};
```

---

## 0.6 Least Squares — Solving Many Equations at Once

### The problem

In SPP you typically have 8–12 satellite measurements but only 4 unknowns (x, y, z, clock). That is **more equations than unknowns** — the system is *over-determined*. No exact solution exists because measurements are noisy. Least squares finds the solution that minimizes the sum of squared errors.

### Setting up the system

Write every equation as:

$$\text{(measured range)}_i = \text{(function of unknowns)}$$

After linearizing (Section 0.9) this becomes:

$$\mathbf{H}\,\mathbf{x} = \mathbf{b} + \boldsymbol{\varepsilon}$$

where:
- $\mathbf{H}$ is the **design matrix** (geometry matrix), $m \times n$
- $\mathbf{x}$ is the vector of unknown corrections, $n \times 1$
- $\mathbf{b}$ is the vector of measured residuals, $m \times 1$
- $\boldsymbol{\varepsilon}$ is the noise vector we want to minimize

### Ordinary Least Squares (OLS)

$$\hat{\mathbf{x}} = (\mathbf{H}^\top \mathbf{H})^{-1} \mathbf{H}^\top \mathbf{b}$$

**Intuition:** $\mathbf{H}^\top \mathbf{b}$ projects the measurements onto the model space. $(\mathbf{H}^\top\mathbf{H})^{-1}$ scales by the geometry.

### Weighted Least Squares (WLS)

Not all measurements are equally reliable. A satellite at 5° elevation passes through much more atmosphere than one at 80°. We give it a smaller weight. The **weight matrix** $\mathbf{W}$ is diagonal:

$$\hat{\mathbf{x}} = (\mathbf{H}^\top \mathbf{W} \mathbf{H})^{-1} \mathbf{H}^\top \mathbf{W} \mathbf{b}$$

In libgnss++ (`spp.cpp`), elevation-dependent weighting uses $\sin^2(el)$:
```cpp
double sin_el = std::sin(geom.elevation);
measurement.weight = sin_el * sin_el;
```

Then the rows of **H** and **b** are scaled by $\sqrt{w_i}$ before solving:
```cpp
weighted_H.row(i) = H.row(i) * sigma;        // sigma = sqrt(weight)
weighted_residuals(i) = residuals(i) * sigma;
```

This converts WLS into an ordinary least-squares problem on the scaled system, solved by QR decomposition:
```cpp
Eigen::ColPivHouseholderQR<MatrixXd> qr(weighted_H);
VectorXd dx = qr.solve(weighted_residuals);
```

### Solution covariance

The uncertainty of the least-squares solution is:

$$\mathbf{P} = \sigma_0^2\,(\mathbf{H}^\top \mathbf{W} \mathbf{H})^{-1}$$

where $\sigma_0^2$ is the variance of unit weight. A large diagonal means a poorly determined component (e.g., height is worse determined than horizontal position).

---

## 0.7 Statistics — Noise, Error, and Uncertainty

### Random vs systematic error

- **Systematic error (bias):** Always in the same direction. Example: your clock is always 1 µs fast. This cannot be removed by averaging more measurements.
- **Random error (noise):** Fluctuates around zero. Example: thermal noise in the receiver. Averaging more measurements *does* help.

### Gaussian (normal) distribution

Most GNSS errors are modelled as Gaussian. A Gaussian with mean 0 and standard deviation σ has the bell-curve shape. **~68%** of samples fall within ±σ, **~95%** within ±2σ, **~99.7%** within ±3σ.

### Root Mean Square (RMS) error

$$\text{RMS} = \sqrt{\frac{1}{n}\sum_{i=1}^{n} e_i^2}$$

libgnss++ reports RMS of the measurement residuals as a quality check:
```cpp
solution.residual_rms = std::sqrt(final_residuals.squaredNorm() /
                                  static_cast<double>(final_measurements.size()));
```

### Chi-squared test

Used to decide if residuals are consistent with the assumed noise model. If residuals are too large, something is wrong (bad satellite, multipath). The RTK engine uses a ratio test (Section 12) to validate the integer ambiguity hypothesis.

---

## 0.8 Iterative Methods

Many GNSS equations are **nonlinear** (they involve square roots of sums of squares). We cannot solve them directly — instead we use iteration: start with a rough guess, improve it step by step until changes become negligibly small.

### Newton's method in 1D

Find *x* such that $f(x) = 0$:
$$x_{k+1} = x_k - \frac{f(x_k)}{f'(x_k)}$$

Each step moves in the direction the function slopes, scaled by how steep it is.

### Newton–Raphson in multiple dimensions

Extend to a vector of unknowns **x** and a vector of equations **f(x)**:
$$\mathbf{x}_{k+1} = \mathbf{x}_k - \mathbf{J}^{-1}\mathbf{f}(\mathbf{x}_k)$$

where $\mathbf{J}$ is the Jacobian matrix of partial derivatives. This is exactly what iterative WLS does in SPP (Section 8).

### Kepler's equation — a famous iterative solve

One of the oldest iterative problems in all of astronomy: find eccentric anomaly *E* from mean anomaly *M* and eccentricity *e*:

$$M = E - e\sin E \quad \Rightarrow \quad E_{k+1} = M + e\sin E_k$$

In libgnss++ (`navigation.cpp`):
```cpp
// solveKepler(): iterate until convergence
```

This is used to propagate satellite orbits from broadcast parameters.

### Convergence criterion

Stop iterating when the correction is small enough. In libgnss++ SPP:
```cpp
if (dx.head<3>().norm() < spp_config_.position_convergence_threshold) {
    // 0.1 mm by default — converged!
    break;
}
```

---

## 0.9 Linearization — Turning Curves into Lines

The pseudorange equation is non-linear in the receiver position. Least squares only works on linear equations. Linearization (first-order Taylor expansion) converts the nonlinear problem to a linear one *near the current estimate*.

### Taylor expansion in 1D

Any smooth function near a point $x_0$:

$$f(x) \approx f(x_0) + f'(x_0)(x - x_0)$$

The curve is approximated by its tangent line at $x_0$.

### In multiple dimensions

$$f(\mathbf{x}) \approx f(\mathbf{x}_0) + \nabla f(\mathbf{x}_0)^\top (\mathbf{x} - \mathbf{x}_0)$$

where $\nabla f$ is the gradient (vector of partial derivatives).

### Applied to GNSS range

The geometric range from receiver $\mathbf{r}$ to satellite $\mathbf{s}$ is:

$$\rho = \|\mathbf{s} - \mathbf{r}\| = \sqrt{(s_x-r_x)^2 + (s_y-r_y)^2 + (s_z-r_z)^2}$$

The partial derivative with respect to $r_x$ (how much range changes if only x moves):

$$\frac{\partial \rho}{\partial r_x} = \frac{-(s_x - r_x)}{\|\mathbf{s}-\mathbf{r}\|} = -\hat{l}_x$$

where $\hat{l}_x$ is the x-component of the unit line-of-sight vector.

So the linearized equation for one satellite is:

$$\delta\rho \approx -\hat{l}_x\,\delta r_x - \hat{l}_y\,\delta r_y - \hat{l}_z\,\delta r_z + c\,\delta t_r$$

This is one row of the design matrix **H** in the SPP solver. In libgnss++ (`spp.cpp`):
```cpp
const Vector3d los = (satellite_position - position).normalized();
H(i, 0) = -los(0);   // ∂ρ/∂x
H(i, 1) = -los(1);   // ∂ρ/∂y
H(i, 2) = -los(2);   // ∂ρ/∂z
H(i, 3) = 1.0;       // ∂ρ/∂(c·δt) — clock column
```

We then iterate: solve for the correction $\delta\mathbf{x}$, update the estimate, rebuild **H** at the new point, repeat.

---

## 0.10 Logarithms and the dB Scale

Logarithms show up every time you work with GNSS signal strength. The `snr` field in every `Observation` is measured in **dB-Hz** — and understanding that unit requires knowing what a logarithm is.

### 0.10.1 What is a Logarithm?

A logarithm is the *inverse* of a power. Ask: "what exponent do I need to produce this number?"

$$\log_{10}(x) = y \quad \Longleftrightarrow \quad 10^y = x$$

Examples:

| Expression         | Answer | Reasoning               |
|--------------------|--------|-------------------------|
| $\log_{10}(100)$   | 2      | $10^2 = 100$            |
| $\log_{10}(1000)$  | 3      | $10^3 = 1000$           |
| $\log_{10}(1)$     | 0      | $10^0 = 1$              |
| $\log_{10}(0.01)$  | $-2$   | $10^{-2} = 0.01$        |
| $\log_{10}(\sqrt{10})$ | 0.5 | $10^{0.5} = \sqrt{10}$ |

**Key intuition:** Every time you multiply the input by 10, the log increases by 1. So log₁₀ compresses enormous ranges into manageable numbers — exactly what you need when comparing signal powers that span factors of $10^{12}$.

### 0.10.2 Log Rules (The Only Three You Need)

$$\log(a \times b) = \log(a) + \log(b) \qquad \text{(product → sum)}$$
$$\log\!\left(\frac{a}{b}\right) = \log(a) - \log(b) \qquad \text{(quotient → difference)}$$
$$\log(a^n) = n \cdot \log(a) \qquad \text{(power → multiply)}$$

These rules are why engineers love logarithms: multiplying gains in a signal chain becomes adding dB values.

### 0.10.3 The Natural Logarithm (ln)

$$\ln(x) = \log_e(x), \quad e \approx 2.71828$$

$e$ is Euler's number, the base used in all growth/decay phenomena (atmospheric pressure with altitude, noise power spectral density). The natural log obeys the same three rules above.

In the Saastamoinen troposphere model (`troposphere.hpp`), pressure drops with altitude following a power law equivalent to an exponential:
```cpp
double P = 1013.25 * std::pow(1.0 - 2.2557e-5 * hgt, 5.2568);
```

### 0.10.4 The Decibel (dB)

A **decibel** is a dimensionless ratio of two power values, expressed on a logarithmic scale:

$$\text{Level (dB)} = 10 \cdot \log_{10}\!\left(\frac{P_2}{P_1}\right)$$

Why the factor of 10? Historical convention ("deci-" = one tenth of a Bel).

**Worked examples:**

| Ratio $P_2/P_1$ | Level in dB | Interpretation           |
|-----------------|-------------|---------------------------|
| $10$            | 10 dB       | 10× more power            |
| $100$           | 20 dB       | 100× more power           |
| $1{,}000$       | 30 dB       | 1000× more power          |
| $2$             | ≈ 3 dB      | double the power          |
| $1$             | 0 dB        | same power                |
| $0.5$           | ≈ −3 dB     | half the power            |
| $0.001$         | −30 dB      | one-thousandth the power  |

**Quick rule:** ±10 dB = ×10 / ÷10 power. ±3 dB ≈ ×2 / ÷2 power.

### 0.10.5 dB-Hz — The SNR Unit in GNSS

The GNSS signal-to-noise ratio is expressed in **dB-Hz** (decibels relative to one hertz). It measures signal power relative to the noise power *density* (noise power per 1 Hz bandwidth):

$$\text{SNR (dB-Hz)} = 10 \cdot \log_{10}\!\left(\frac{S}{N_0}\right)$$

where $S$ is the signal power (Watts) and $N_0$ is the noise power spectral density (W/Hz).

In `observation.hpp`:
```cpp
double snr = 0.0;    ///< Signal-to-noise ratio in dB-Hz
```

**Typical GNSS values:**

| SNR (dB-Hz) | Signal quality         | Typical condition          |
|-------------|------------------------|----------------------------|
| 50–55       | Excellent              | Open sky, low elevation fine |
| 40–50       | Good                   | Normal open-sky operation  |
| 30–40       | Marginal               | Tree canopy, shallow angles |
| 25–30       | Poor                   | Urban canyon, indoor       |
| < 25        | Unusable (usually)     | Too noisy to track reliably |

libgnss++ uses SNR as a quality gate (configurable `snr_mask`):
```cpp
// spp.cpp
if (observation.snr < config_.snr_mask) {
    continue;   // reject this satellite
}
```

### 0.10.6 Converting dB-Hz Back to a Linear Ratio

Given SNR in dB-Hz, recover the linear $S/N_0$:

$$\frac{S}{N_0} = 10^{\,\text{SNR}_{\text{dB-Hz}} / 10}$$

Example: SNR = 45 dB-Hz → $S/N_0 = 10^{4.5} \approx 31{,}623$ Hz.

### 0.10.7 Why Elevation and SNR Are Related

A satellite near the horizon has its signal passing through much more atmosphere. More atmosphere = more signal absorption = lower signal power arriving at the receiver = lower SNR. This is why both the **elevation mask** (reject below 15°) and the **SNR mask** (reject below threshold) are applied:

```
Elevation 5°  → long atmospheric path → low SNR (maybe 25 dB-Hz)
Elevation 45° → shorter path        → good SNR (maybe 45 dB-Hz)
Elevation 90° → shortest path       → best SNR (maybe 52 dB-Hz)
```

The elevation-dependent weighting in WLS uses $\sin^2(el)$ rather than SNR directly because SNR can be affected by multipath and hardware gains, while elevation is a clean geometric quantity. Both serve the same purpose: down-weight noisy measurements.

---

# Part 1 — GNSS Positioning

---

## 1. What is GNSS? (Big Picture)

**GNSS** stands for **Global Navigation Satellite System**. The idea is elegantly simple:

> A constellation of satellites broadcast radio signals. A receiver on the ground listens to those signals, measures how long they took to arrive, multiplies by the speed of light, and uses those distances to compute its own position.

### The four major constellations

| System   | Country   | Satellites | Notes                          |
|----------|-----------|------------|--------------------------------|
| GPS      | USA       | 31         | Original system, 1978–present  |
| GLONASS  | Russia    | 24         | Uses frequency-division signals|
| Galileo  | EU        | 30         | Civilian-focus, modern         |
| BeiDou   | China     | 35         | Also has GEO satellites        |

Additionally: QZSS (Japan, regional), NavIC (India, regional), SBAS (augmentation).

libgnss++ supports all of these. In `types.hpp`:
```cpp
enum class GNSSSystem : uint8_t {
    GPS, GLONASS, Galileo, BeiDou, QZSS, SBAS, NavIC
};
```

### Why do we need at least 4 satellites?

You have 4 unknowns: position (x, y, z) + receiver clock offset. Each satellite gives you one range equation. You need 4 equations to solve for 4 unknowns. More satellites → over-determined system → better accuracy via least squares.

### The accuracy spectrum

| Mode   | Technique                          | Typical Accuracy         |
|--------|------------------------------------|--------------------------|
| SPP    | Pseudorange, broadcast products    | 1–5 m (open sky)         |
| DGNSS  | Differential pseudorange           | 0.3–1 m                  |
| RTK    | Carrier phase + base station        | 1–5 cm (fixed)           |
| PPP    | Carrier phase + precise products   | 2–10 cm after convergence|

---

## 2. The World of Signals and Frequencies

### Radio waves: frequency, wavelength, and the speed of light

All GNSS signals travel at the speed of light $c = 299\,792\,458 \text{ m/s}$.

A radio wave oscillates at a frequency $f$ (cycles per second, Hz). The distance one full cycle occupies in space is the **wavelength**:

$$\lambda = \frac{c}{f}$$

In libgnss++ (`constants.hpp`):
```cpp
constexpr double GPS_L1_FREQ       = 1575.42e6;   // 1575.42 MHz
constexpr double GPS_L1_WAVELENGTH = SPEED_OF_LIGHT / GPS_L1_FREQ;  // ≈ 0.1903 m
```

The wavelength of GPS L1 is about **19 cm**. This will matter enormously when we talk about carrier phase in Section 5.

### Signal bands

| Signal  | Frequency       | Wavelength | Used for                     |
|---------|-----------------|------------|------------------------------|
| GPS L1  | 1575.42 MHz     | 19.03 cm   | Primary civilian signal      |
| GPS L2  | 1227.60 MHz     | 24.42 cm   | Dual-frequency combinations  |
| GPS L5  | 1176.45 MHz     | 25.48 cm   | Safety-of-life, modernized   |
| GAL E1  | 1575.42 MHz     | 19.03 cm   | Same as GPS L1               |
| BDS B1I | 1561.098 MHz    | 19.20 cm   | BeiDou primary               |
| GLO L1  | ~1602 MHz ± n×0.5625 MHz | varies | GLONASS uses different freq per satellite |

### Why two frequencies?

Ionospheric delay is **proportional to 1/f²**. If you measure on two frequencies, you can *eliminate* the ionospheric error mathematically (ionosphere-free combination). This is used in RTK and PPP.

The ionosphere-free (IF) combination coefficients are defined in `constants.hpp`:
```cpp
// IFLC_C1 = f2² / (f2² - f1²) ≈ 2.546
constexpr double IFLC_C1 = GPS_L2_WAVELENGTH * GPS_L2_WAVELENGTH /
    (GPS_L2_WAVELENGTH * GPS_L2_WAVELENGTH - GPS_L1_WAVELENGTH * GPS_L1_WAVELENGTH);
// IFLC_C2 = -f1² / (f2² - f1²) ≈ -1.546
constexpr double IFLC_C2 = -(GPS_L1_WAVELENGTH * GPS_L1_WAVELENGTH) /
    (GPS_L2_WAVELENGTH * GPS_L2_WAVELENGTH - GPS_L1_WAVELENGTH * GPS_L1_WAVELENGTH);
```

The ionosphere-free pseudorange: $P_{IF} = C_1 \cdot P_{L1} + C_2 \cdot P_{L2}$

### Wide-lane and narrow-lane combinations

Combining signals from two frequencies creates virtual signals with different wavelengths, used in ambiguity resolution:

$$\lambda_{WL} = \frac{c}{f_1 - f_2} \approx 86.2 \text{ cm} \quad \text{(wide-lane, easier to fix)}$$
$$\lambda_{NL} = \frac{c}{f_1 + f_2} \approx 10.7 \text{ cm} \quad \text{(narrow-lane, precise)}$$

```cpp
// constants.hpp
constexpr double GPS_WL_WAVELENGTH = SPEED_OF_LIGHT / (GPS_L1_FREQ - GPS_L2_FREQ);
```

---

## 3. How We Describe Position on Earth

### Three coordinate systems in libgnss++

#### 3.1 ECEF — Earth-Centred Earth-Fixed

The **origin** is the Earth's centre of mass. The x-axis points toward the prime meridian (0° longitude) at the equator. The z-axis points toward the North Pole. The y-axis completes the right-hand system.

```
z ↑ (North Pole)
  |
  |
  o-------→ x  (prime meridian, equator)
 /
↗ y  (90°E, equator)
```

All satellite positions and receiver positions are computed in ECEF meters inside libgnss++. When you see `Vector3d pos` in the code, it is almost always ECEF.

#### 3.2 Geodetic — Latitude, Longitude, Height

The human-readable format: latitude φ (degrees N/S), longitude λ (degrees E/W), height h above the **WGS-84 ellipsoid** (roughly sea level but not exactly).

The conversion from ECEF $(x,y,z)$ to geodetic requires solving a nonlinear equation. libgnss++ uses an iterative method in `coordinates.hpp`:

```cpp
inline void ecef2geodetic(const Vector3d& ecef, double& lat, double& lon, double& h) {
    double p = std::sqrt(ecef(0)*ecef(0) + ecef(1)*ecef(1));
    lon = std::atan2(ecef(1), ecef(0));  // longitude is easy

    // latitude requires iteration because Earth is an ellipsoid
    double z = ecef(2), zk = 0.0, v = a;
    for (int k = 0; k < 10 && std::abs(z - zk) >= 1e-4; ++k) {
        zk = z;
        double sinp = z / std::sqrt(p*p + z*z);
        v = a / std::sqrt(1.0 - e2 * sinp * sinp);  // radius of curvature
        z = ecef(2) + v * e2 * sinp;
    }
    lat = std::atan2(z, p);
    h   = std::sqrt(p*p + z*z) - v;
}
```

**From geodetic back to ECEF** is direct (no iteration):
$$x = (N + h)\cos\phi\cos\lambda$$
$$y = (N + h)\cos\phi\sin\lambda$$
$$z = (N(1-e^2) + h)\sin\phi$$

where $N = a / \sqrt{1 - e^2 \sin^2\phi}$ is the radius of curvature in the prime vertical, $a = 6{,}378{,}137$ m is the WGS-84 semi-major axis, and $e^2 \approx 0.006694$ is the first eccentricity squared.

#### 3.3 ENU — East-North-Up (Local Frame)

A local coordinate system centred at a reference point. Easy for humans: East = x, North = y, Up = z. Used when expressing position errors or velocity.

The conversion uses a rotation matrix that depends on origin latitude and longitude:
```cpp
// coordinates.hpp
inline Vector3d ecef2enu(const Vector3d& ecef_diff, double lat, double lon) {
    // standard rotation matrix
    return Vector3d(
        -sinlon * ecef_diff(0) + coslon * ecef_diff(1),
        -sinlat*coslon*ecef_diff(0) - sinlat*sinlon*ecef_diff(1) + coslat*ecef_diff(2),
         coslat*coslon*ecef_diff(0) +  coslat*sinlon*ecef_diff(1) + sinlat*ecef_diff(2)
    );
}
```

### The WGS-84 Ellipsoid

The Earth is not a sphere — it bulges at the equator. The **WGS-84** reference ellipsoid models this:

| Parameter  | Value                | Meaning                       |
|------------|----------------------|-------------------------------|
| a          | 6,378,137.0 m        | Semi-major axis (equatorial)   |
| f          | 1/298.257223563      | Flattening                    |
| b = a(1−f) | 6,356,752.3 m        | Semi-minor axis (polar)        |
| e²         | 0.006694379990       | First eccentricity squared    |

Defined in `constants.hpp`:
```cpp
constexpr double WGS84_A  = 6378137.0;
constexpr double WGS84_F  = 1.0 / 298.257223563;
constexpr double WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F;
```

---

## 4. GNSS Time Systems

### Why time is so critical

The fundamental measurement in GNSS is the **travel time** of a signal from satellite to receiver. That time, multiplied by $c$, gives the range. A 1 µs (microsecond) timing error = 300 m position error. A 1 ns (nanosecond) error = 30 cm. You need sub-nanosecond accuracy.

### GPS Time

GPS uses its own time scale called **GPS Time (GPST)**. It is measured as:
- **GPS week number** — integer count of 7-day weeks since 6 January 1980.
- **Time of week (TOW)** — seconds elapsed since the start of the current week (0–604,799 s).

```cpp
struct GNSSTime {
    int week;    // GPS week number
    double tow;  // time of week in seconds
};
constexpr double SECONDS_PER_WEEK = 604800.0;  // 7 * 24 * 3600
```

### GPS Time vs UTC

GPS Time does **not** have leap seconds. UTC (civil time) does. As of 2024, GPS time is 18 seconds ahead of UTC. Receivers must handle this offset.

### Satellite vs receiver clock

Each satellite has an onboard atomic clock (accurate to ~10 ns), but it has a small drift and offset. The satellite broadcasts three polynomial coefficients (af0, af1, af2) so you can correct for this:

$$\delta t^{sat} = a_{f0} + a_{f1}(t - t_{oc}) + a_{f2}(t - t_{oc})^2 + \delta t_{rel}$$

where $t_{oc}$ is the clock reference time, and $\delta t_{rel}$ is the relativistic correction (orbit-velocity-dependent, about −38 µs/day for GPS satellites). From `navigation.cpp`:
```cpp
clock_bias = eph.af0 + eph.af1 * tc + eph.af2 * tc * tc;
clock_bias += kRelativityF * eph.e * eph.sqrt_a * sin_e;  // relativistic term
```

The receiver clock is cheap and has a large unknown offset — this is the fourth unknown in the SPP solution.

### Signal transmission time and the Sagnac effect

The satellite position must be computed at the **time the signal was transmitted**, not received. The Earth rotates during the signal's ~70 ms travel. libgnss++ corrects for this (**Sagnac/Earth-rotation correction**) in `coordinates.hpp`:

```cpp
inline double geodist(const Vector3d& sat_pos, const Vector3d& rec_pos) {
    double r = (sat_pos - rec_pos).norm();
    return r + constants::OMEGA_E *
        (sat_pos(0)*rec_pos(1) - sat_pos(1)*rec_pos(0)) / constants::SPEED_OF_LIGHT;
}
```

The extra term $\frac{\Omega_E}{c}(x^s y^r - y^s x^r)$ is about 30 m for a 70 ms travel time — not negligible!

Also applied in `spp.cpp` via a rotation matrix:
```cpp
double angle = constants::OMEGA_E * signal_travel;
Eigen::Matrix3d earth_rotation;
earth_rotation << cos(angle), sin(angle), 0,
                 -sin(angle), cos(angle), 0,
                  0,          0,          1;
Vector3d corrected_sat_pos = earth_rotation * sat_pos;
```

---

## 5. What the Receiver Measures (Observations)

A GNSS receiver produces two types of measurements for each satellite signal:

### 5.1 Pseudorange (Code Measurement)

The receiver generates a local copy of the satellite's PRN (pseudo-random noise) code and shifts it in time until it aligns with the received signal. The shift × speed of light = range. It is called *pseudo*range because it includes the receiver clock offset.

$$P = \rho + c(\delta t_r - \delta t^s) + I + T + \varepsilon_P$$

| Symbol       | Meaning                            | Typical size  |
|--------------|------------------------------------|---------------|
| $\rho$       | True geometric range               | 20,000–26,000 km |
| $c\,\delta t_r$ | Receiver clock bias × c        | can be millions of meters |
| $c\,\delta t^s$ | Satellite clock error × c     | ~10 m          |
| $I$          | Ionospheric delay                  | 1–50 m         |
| $T$          | Tropospheric delay                 | 2–25 m         |
| $\varepsilon_P$ | Code noise + multipath         | 0.3–3 m        |

In libgnss++ (`observation.hpp`):
```cpp
double pseudorange = 0.0;   // in meters
bool has_pseudorange = false;
```

### 5.2 Carrier Phase Measurement

The receiver also counts how many complete cycles of the carrier wave have been received, plus the fractional part. Since the wavelength is ~19 cm, this measurement is **much more precise** than the pseudorange.

$$\Phi = \rho + c(\delta t_r - \delta t^s) + \lambda N - I + T + \varepsilon_\Phi$$

The key difference: $\lambda N$ is the **integer ambiguity**. At the moment the receiver starts tracking, it does not know how many whole wavelengths of signal are in flight. This integer $N$ is unknown and must be resolved (RTK/PPP). This is the hardest problem in precision GNSS.

**Note the sign of ionosphere:** it is $+I$ for pseudorange (delay) but $-I$ for carrier phase (the ionosphere advances the phase — it is a medium with refractive index < 1 for phase). This provides dual-frequency ionosphere cancellation.

```cpp
double carrier_phase = 0.0;    // in cycles (not meters!)
bool has_carrier_phase = false;
```

Converting cycles to meters: multiply by wavelength $\lambda$.

### 5.3 Doppler Frequency

The receiver measures the instantaneous frequency shift of the received signal due to relative motion between satellite and receiver (Doppler effect). Used for velocity estimation and cycle-slip detection.

```cpp
double doppler = 0.0;    // in Hz
bool has_doppler = false;
```

### 5.4 Signal-to-Noise Ratio (SNR)

The ratio of signal power to noise power, in dB-Hz. Higher is better. Signals below ~25 dB-Hz are generally unusable. The elevation mask (default 15°) rejects low-angle signals that have low SNR and more atmospheric error.

```cpp
double snr = 0.0;    // in dB-Hz
```

### 5.5 Loss of Lock (Cycle Slip)

If the receiver briefly loses signal lock (a building, a tree, high dynamics), the carrier phase count resets. The phase appears to "jump" by a non-integer number of cycles. This is a **cycle slip** and must be detected. libgnss++ uses three detectors in `rtk_slip_detection.hpp`:
- Doppler-based detection
- Code–carrier divergence detection
- Phase difference between epochs

---

## 6. Where Are the Satellites? (Ephemeris)

To compute a position, you need to know where each satellite is. This information is called **ephemeris** and is broadcast by each satellite in its navigation message.

### 6.1 Orbital Elements (Keplerian Parameters)

A satellite orbit is an ellipse. It is described by 6 classical Keplerian parameters plus correction terms.

#### The shape of the orbit

| Parameter | Symbol  | Meaning                                                         |
|-----------|---------|-----------------------------------------------------------------|
| Semi-major axis | $a = (\sqrt{a})^2$ | Size of the orbital ellipse |
| Eccentricity | $e$    | How elongated (0 = circle, 1 = parabola)                    |

In the broadcast message, $\sqrt{a}$ is given directly:
```cpp
double sqrt_a;    // square root of semi-major axis
double e;         // eccentricity
```

#### The orientation of the orbit

| Parameter       | Symbol    | Meaning                                             |
|-----------------|-----------|-----------------------------------------------------|
| Inclination     | $i_0$     | Tilt of orbital plane relative to equator            |
| Right Ascension | $\Omega_0$| Longitude of ascending node (where orbit crosses equator going north) |
| Argument of perigee | $\omega$ | Angle in orbital plane to the closest point     |
| Mean anomaly at epoch | $M_0$ | Where the satellite is on its orbit at reference time |

```cpp
double i0;        // inclination at toe
double omega0;    // right ascension of ascending node
double omega;     // argument of perigee
double m0;        // mean anomaly at toe
double idot;      // rate of change of inclination
double omega_dot; // rate of change of right ascension
double delta_n;   // mean motion correction
```

#### Perturbation corrections

Real orbits are not perfect ellipses — the Earth is not a perfect sphere, the Sun and Moon exert forces. Six small corrections are broadcast:

```cpp
double cuc, cus;  // cosine/sine corrections to argument of latitude
double crc, crs;  // cosine/sine corrections to orbital radius
double cic, cis;  // cosine/sine corrections to inclination
```

### 6.2 Computing Satellite Position Step by Step

Here is the complete algorithm as implemented in `navigation.cpp`:

**Step 1: Compute semi-major axis**
$$a = (\sqrt{a})^2$$

**Step 2: Compute time from reference epoch**
$$t_k = t - t_{oe}$$
(normalized to ±302,400 s)

**Step 3: Compute mean motion and mean anomaly**
$$n_0 = \sqrt{\frac{\mu}{a^3}}, \quad n = n_0 + \Delta n$$
$$M = M_0 + n \cdot t_k$$
where $\mu = 3.986005 \times 10^{14}$ m³/s² is Earth's gravitational constant.

**Step 4: Solve Kepler's equation** (iteratively!)
$$M = E - e \sin E \quad \Rightarrow \quad E$$

**Step 5: Compute true anomaly**
$$\nu = \arctan\!\left(\frac{\sqrt{1-e^2}\sin E}{\cos E - e}\right)$$

**Step 6: Argument of latitude and corrections**
$$\phi = \nu + \omega$$
$$u = \phi + C_{us}\sin(2\phi) + C_{uc}\cos(2\phi)$$
$$r = a(1 - e\cos E) + C_{rs}\sin(2\phi) + C_{rc}\cos(2\phi)$$
$$i = i_0 + \dot{i}\,t_k + C_{is}\sin(2\phi) + C_{ic}\cos(2\phi)$$

**Step 7: Longitude of ascending node**
$$\Omega = \Omega_0 + (\dot{\Omega} - \dot{\Omega}_E)\,t_k - \dot{\Omega}_E\,t_{oe}$$
where $\dot{\Omega}_E = 7.2921151467 \times 10^{-5}$ rad/s is Earth's rotation rate.

**Step 8: Position in orbital plane**
$$x' = r\cos u, \quad y' = r\sin u$$

**Step 9: Convert to ECEF**
$$x = x'\cos\Omega - y'\cos i\sin\Omega$$
$$y = x'\sin\Omega + y'\cos i\cos\Omega$$
$$z = y'\sin i$$

In `navigation.cpp`:
```cpp
const double n  = n0 + eph.delta_n;
const double mean_anomaly = eph.m0 + n * tk;
const double eccentric_anomaly = solveKepler(mean_anomaly, eph.e);
// ... true anomaly, corrections ...
pos(0) = x_orb * cos_omega - y_orb * cos_i * sin_omega;
pos(1) = x_orb * sin_omega + y_orb * cos_i * cos_omega;
pos(2) = y_orb * sin_i;
```

### 6.3 Clock Correction

$$\delta t^s = a_{f0} + a_{f1}(t - t_{oc}) + a_{f2}(t - t_{oc})^2 + \delta t_{rel}$$

Relativistic term: $\delta t_{rel} = F \cdot e \cdot \sqrt{a} \cdot \sin E$, where $F = -4.442807633 \times 10^{-10}$ s/√m.

The satellite clock bias in meters: $\delta t^s \times c$ ≈ several meters, must be corrected.

### 6.4 GLONASS: Different Ephemeris Format

GLONASS satellites broadcast their **state vector** (position + velocity + acceleration) directly, rather than Keplerian elements. The future position is computed by numerically integrating the equations of motion using a 4th-order Runge-Kutta method:

```cpp
void propagateGlonassOrbit(double dt, double* state, const Vector3d& acceleration) {
    double k1[6], k2[6], k3[6], k4[6], work[6];
    // Runge-Kutta 4th order integration
    computeGlonassDynamics(state, k1, acceleration);
    // ...
    for (int i = 0; i < 6; ++i) {
        state[i] += (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]) * dt / 6.0;
    }
}
```

The dynamics include: gravitational acceleration, J2 oblateness correction, Coriolis, centrifugal, and luni-solar forces.

---

## 7. Error Sources: What Corrupts Our Measurements

Understanding errors is critical for building a good GNSS engine. Here are the main error sources and how libgnss++ handles them.

### 7.1 Ionospheric Delay

The ionosphere (60–1000 km altitude) is a layer of free electrons. Radio signals slow slightly as they pass through it. The delay is:

$$I = \frac{40.3 \cdot \text{TEC}}{f^2}$$

where TEC = Total Electron Content in TECU (1 TECU = $10^{16}$ electrons/m²), and $f$ is the frequency in Hz.

**Key property:** proportional to $1/f^2$ → higher frequency = less delay.

**For SPP (single frequency):** use the **Klobuchar model** — an 8-parameter model broadcast in the GPS navigation message that corrects ~50% of the ionospheric error.

The model computes the delay at the ionosphere pierce point (a virtual point where the signal line crosses the ionosphere at ~350 km altitude):

```cpp
// ionosphere.hpp — Klobuchar model
inline double ionoDelayKlobuchar(double lat, double lon, double az, double el,
                                  double tow, const double* alpha, const double* beta) {
    // Compute ionosphere pierce point
    double psi = 0.0137 / (el * SC / M_PI + 0.11) - 0.022;  // earth-centred angle
    double phi_i = phi_u + psi * cos(az);   // sub-ionospheric latitude
    // ...
    // Polynomial model: amplitude and period vary with geomagnetic latitude
    double AMP = a[0] + a[1]*phi_m + a[2]*phi_m*phi_m + a[3]*phi_m*phi_m*phi_m;
    double PER = b[0] + b[1]*phi_m + b[2]*phi_m*phi_m + b[3]*phi_m*phi_m*phi_m;
    // Cosine day-time variation + constant 5 ns night-time floor
    ...
}
```

**For RTK (dual-frequency):** double-differencing between nearby stations mostly cancels the ionosphere. Long baselines may require the ionosphere-free combination.

**For PPP:** use the ionosphere-free combination, or IONEX maps, or SSR corrections.

### 7.2 Tropospheric Delay

The troposphere (0–50 km) is non-dispersive — the delay does **not** depend on frequency. This makes it harder to remove with dual-frequency. The delay is modelled as:

$$T = T_z \cdot M(el)$$

where $T_z$ is the **zenith tropospheric delay** (ZTD) and $M(el)$ is the **mapping function** that scales it to the elevation angle.

**Zenith delay** has two components:
- **Hydrostatic (dry):** ~2.3 m at sea level, very predictable from surface pressure.
- **Wet:** ~0–30 cm, highly variable with humidity.

**Saastamoinen model** (used in SPP/RTK in libgnss++):
```cpp
// troposphere.hpp
inline double tropDelaySaastamoinen(const Vector3d& pos_ecef, double elevation) {
    double T = 15.0 - 6.5e-3 * hgt + 273.16;  // temperature [K]
    double P = 1013.25 * pow(1.0 - 2.2557e-5 * hgt, 5.2568);  // pressure [mbar]
    double e = 6.108 * 0.7 * exp((17.15 * T - 4684.0) / (T - 38.45));  // water vapour

    // Hydrostatic term
    double trph = 0.0022768 * P /
        (1.0 - 0.00266 * cos(2.0 * lat) - 0.00028 * hgt * 1e-3) / cos_z;
    // Wet term
    double trpw = 0.002277 * (1255.0 / T + 0.05) * e / cos_z;
    return trph + trpw;
}
```

**Niell Mapping Function (NMF)** (for PPP): more accurate, latitude- and season-dependent:
```cpp
// troposphere.hpp — niellHydrostaticMapping()
double mapping = niellMappingContinuedFraction(sin_elevation, a, b, c);
```

### 7.3 Receiver Clock Offset

The receiver has an inexpensive quartz oscillator. Its offset from GPS time may be millions of microseconds. This is estimated as the 4th unknown in the position solution. After SPP, it is typically kept as an a-priori for the next epoch.

### 7.4 Satellite Clock Error

Even atomic clocks drift slightly. The polynomial correction in the ephemeris reduces this to ~1 ns (~30 cm). For PPP, precise clock products reduce it further to ~0.05 ns (~1.5 cm).

### 7.5 Multipath

The receiver receives the direct signal *and* reflections from nearby surfaces (buildings, the ground). The reflected path is longer. Effect: pseudorange error up to ~10–30 m in urban canyons. Carrier phase multipath is smaller (~1–5 cm). Difficult to model — the primary mitigation is the elevation mask (ignore satellites < 15°).

### 7.6 Phase Wind-Up

As the satellite rotates in orbit, the polarization of its circularly polarized signal rotates (wind-up). This causes a phase offset of up to 1 cycle if satellite or receiver rotates 360°. Corrected in PPP using the satellite yaw attitude model.

### 7.7 Relativistic Effects

Two effects from special and general relativity:
1. **Orbital eccentricity term:** The satellite moves faster at perigee (faster = slower clock in SR), and is deeper in gravity well at perigee (deeper = slower clock in GR). The combined periodic effect: $\delta t_{rel} = F \cdot e \cdot \sqrt{a} \cdot \sin E$ — already included in Section 6.2.
2. **Sagnac effect:** Earth rotation during signal travel — covered in Section 4.

### 7.8 Antenna Phase Centre Offset (PCO/PCV)

The physical connector of the antenna is not where the phase centre is. The phase centre also varies with elevation (Phase Centre Variation, PCV). For cm-level accuracy these millimetre-level corrections matter. libgnss++ supports ANTEX-format antenna calibration files.

### 7.9 Group Delay (TGD)

The GPS satellite hardware introduces a small differential bias between L1 and L2 (~10 ns). For single-frequency SPP this must be corrected:

$$\delta_{TGD} = T_{GD} \times c$$

```cpp
// spp.cpp
double corrected_pr = obs.pseudorange
                      + sat_clk * SPEED_OF_LIGHT
                      - trop_delay
                      - iono_delay
                      - groupDelayCorrectionMeters(obs, *eph);  // TGD correction
```

---

## 8. SPP — Single Point Positioning

SPP uses only pseudorange measurements and broadcast ephemeris from a single receiver. It is the baseline mode used in all smartphones, cars, and handheld devices.

### 8.1 The Observation Equation

For satellite $s$ and receiver $r$:

$$P_r^s = \rho_r^s + c(\delta t_r - \delta t^s) + I_r^s + T_r^s + \varepsilon_r^s$$

After applying all corrections (satellite clock, ionosphere, troposphere, TGD), the corrected pseudorange is:

$$\tilde{P}_r^s = P_r^s + c\,\delta t^s - I - T - \delta_{TGD} \approx \rho_r^s + c\,\delta t_r$$

This leaves only geometry $\rho$ and the receiver clock bias $c\,\delta t_r$ as unknowns.

### 8.2 Iterative Weighted Least Squares Algorithm

**State vector:** $\mathbf{x} = [\Delta x, \Delta y, \Delta z, c\,\delta t_r]^\top$ (corrections to current estimate)

**Algorithm:**

```
1. Initialize: x0 from a rough position (ECEF centre of Earth or RINEX header)
2. For each iteration:
   a. For each satellite: compute satellite position at transmission time
   b. Apply Earth-rotation (Sagnac) correction to satellite position
   c. Compute elevation angle; skip if < 15°
   d. Apply troposphere correction (Saastamoinen)
   e. Apply ionosphere correction (Klobuchar)
   f. Apply satellite clock and TGD correction
   g. Form corrected pseudorange
   h. Build row of design matrix H:
         H[i] = [-lx, -ly, -lz, 1]
      where (lx,ly,lz) = unit vector from receiver to satellite
   i. Residual = corrected_PR - (geometric_range + clock_bias)
3. Solve WLS: dx = (HᵀWH)⁻¹ HᵀW·residuals
4. Update position and clock: x += dx[:3], clock += dx[3]
5. If ||dx[:3]|| < threshold (1e-4 m): stop
```

In `spp.cpp`:
```cpp
for (int iter = 0; iter < spp_config_.max_iterations; ++iter) {
    auto measurements = buildMeasurements(position);
    // ... build H and residuals ...
    Eigen::ColPivHouseholderQR<MatrixXd> qr(weighted_H);
    VectorXd dx = qr.solve(weighted_residuals);
    position += dx.head<3>();
    clock_bias += dx(3);
    if (dx.head<3>().norm() < spp_config_.position_convergence_threshold) break;
}
```

### 8.3 Multi-Constellation Inter-System Biases

GPS and Galileo have the same reference clock, but GLONASS, BeiDou have their own time scales. Each extra constellation adds one more "clock bias" unknown. libgnss++ handles this by grouping satellites and adding one extra bias column per non-GPS constellation:

```cpp
const int num_unknowns = 4 + static_cast<int>(bias_groups.size());
// bias_groups = [GLONASS, BeiDou] → 6 unknowns total
```

This means you need more satellites to solve the expanded system, but accuracy improves because more observations are used.

### 8.4 Worked Example (Conceptual)

Suppose you have 6 satellites: G01, G08, G15, E12, R03, R07.
- 3 GPS → reference clock group
- 1 Galileo → treated as GPS (same timescale)
- 2 GLONASS → adds 1 GLONASS bias unknown

Total unknowns: 5 (x, y, z, GPS_clock, GLONASS_bias)
Design matrix H: 6 × 5

After WLS: you get ΔECEF position, receiver clock offset (in meters), and GLONASS inter-system bias.

### 8.5 Expected Accuracy

| Condition           | Expected CEP (50th percentile) |
|---------------------|-------------------------------|
| Open sky, GPS+GNSS  | 1–3 m                          |
| Urban canyon        | 3–15 m (multipath)             |
| Poor geometry (high PDOP) | 5–20 m                  |
| Ionosphere storm    | up to 50 m                    |

### 8.6 SPP Data Flow in libgnss++

```
ObservationData (RINEX obs)
       ↓
SPPProcessor::processEpoch()
       ↓
validateObservations()    → filter valid, has_pseudorange, has ephemeris
       ↓
solvePositionLS()
  ├─ buildMeasurements()  → satellite positions, Sagnac, troposphere, ionosphere, TGD
  ├─ build H matrix       → line-of-sight unit vectors
  ├─ WLS solve            → QR decomposition
  └─ ecef2geodetic()      → convert result to lat/lon/height
       ↓
PositionSolution (ECEF + geodetic + DOP + residuals)
```

---

## 9. RTK — Real-Time Kinematic

RTK achieves centimetre accuracy by using **carrier phase measurements** and a **base station** at a known location. By differencing observations between rover and base, most errors cancel.

### 9.1 The Carrier Phase Observation Equation

$$\Phi_r^s = \frac{1}{\lambda}(\rho_r^s + c\,\delta t_r - c\,\delta t^s - I_r^s + T_r^s) + N_r^s + \varepsilon_\Phi$$

where $N_r^s$ is the integer ambiguity (unknown number of whole wavelengths in flight when tracking started).

### 9.2 Differencing Strategy

**Single difference (SD)** between base (*b*) and rover (*r*) for same satellite $s$:

$$\Phi_r^s - \Phi_b^s = \frac{1}{\lambda}(\rho_r^s - \rho_b^s) + c(\delta t_r - \delta t_b) + (N_r^s - N_b^s) + \cdots$$

Satellite clock error $\delta t^s$ cancels perfectly.

**Double difference (DD)** between two satellites $s$ and $p$:

$$(\Phi_r^s - \Phi_b^s) - (\Phi_r^p - \Phi_b^p) = \frac{1}{\lambda}[(\rho_r^s - \rho_b^s) - (\rho_r^p - \rho_b^p)] + \nabla\Delta N + \cdots$$

Now *both* satellite and receiver clocks cancel! Atmospheric errors are also small if the baseline is short (< 10 km), because both stations see nearly the same atmosphere.

The double-difference integer ambiguity $\nabla\Delta N = (N_r^s - N_b^s) - (N_r^p - N_b^p)$ is an integer.

### 9.3 RTK State Vector and Kalman Filter

RTK maintains a **Kalman filter state**:

$$\mathbf{x} = [\underbrace{\Delta x,\Delta y,\Delta z}_{\text{3D baseline}},\; \underbrace{N_1^{sd,1}, N_1^{sd,2}, \ldots}_{\text{SD L1 ambiguities}},\; \underbrace{N_2^{sd,1},\ldots}_{\text{SD L2 ambiguities}}]$$

**State vector size:** 3 + 2×num_satellites (in the RTKLIB-compatible formulation used in libgnss++, single-difference ambiguities are stored; double differences are formed in the measurement model).

The RTK architecture (`rtk.hpp`):
```cpp
/*
 * State vector: [baseline(3), N1_SD_sat1, N1_SD_sat2, ..., N2_SD_sat1, ...]
 * DD formed only in the observation model (H matrix maps SD states).
 * RTKLIB-compatible approach.
 */
```

### 9.4 The Measurement Update

For each epoch:

1. **Select reference satellite** per constellation (highest elevation)
2. **Form DD pairs** (rover–base, sat–ref_sat)
3. **Build DD design matrix** from line-of-sight vectors
4. **Build DD residuals** for code and carrier phase
5. **Apply Kalman filter update** (`kalman.hpp`)

The Kalman filter in libgnss++ implements the RTKLIB-compatible formulation:
```cpp
// kalman.hpp
inline int kalmanFilter(VectorXd& x, MatrixXd& P,
                        const MatrixXd& H, const VectorXd& v, const MatrixXd& R) {
    // Active states only (where x[i] != 0 and P[i,i] > 0)
    // F = P * H'
    // Q = H * P * H' + R
    // K = F * Q^{-1}    (Kalman gain)
    // x = x + K * v     (state update)
    // P = (I - K*H) * P (covariance update)
}
```

### 9.5 Cycle Slip Detection

Before each epoch, libgnss++ checks for cycle slips (`rtk_slip_detection.hpp`). A cycle slip resets the ambiguity for that satellite:

- **Doppler check:** if the Doppler-predicted phase change disagrees with the measured phase change by more than threshold, flag a slip.
- **Code–carrier check:** if $P - \lambda\Phi$ changes rapidly, flag a slip.
- **Melbourne-Wübbena** (code + phase wide-lane combination): robust geometry-free slip detector.

### 9.6 Ambiguity Resolution

After the Kalman filter gives a **float solution** (real-valued ambiguity estimates), we try to find the integer solution using the **LAMBDA algorithm** (Section 12).

Decision steps:
1. Extract ambiguity estimates $\hat{\mathbf{N}}$ and covariance $\mathbf{Q}_N$
2. Run LAMBDA → get best integer candidate $\check{\mathbf{N}}$ and ratio $r$
3. **Ratio test:** if $r > $ threshold (default 3.0) → accept as **FIXED**
4. Update baseline using the fixed integers

```cpp
// rtk.hpp configuration
double ratio_threshold = 3.0;        // LAMBDA ratio threshold
int min_satellites_for_ar = 5;       // need at least 5 sats to fix
int min_lock_count = 5;              // must track for 5 epochs before trying
```

### 9.7 Fix, Float, and Hold

- **FLOAT:** Kalman filter converging, ambiguities not yet resolved. Accuracy ~10–50 cm.
- **FIXED:** Integer ambiguities resolved. Accuracy ~1–5 cm.
- **HOLD:** Once fixed, the solution is "held" to prevent thrashing between fix/float.

The libgnss++ extended AR policy includes additional robustness:
```cpp
int min_hold_count = 5;   // consecutive fixes before holdamb
double hold_ambiguity_ratio_threshold = 2.0;
double max_position_jump_m = 5.0;  // reject wrong-fix with large jumps
```

### 9.8 RTK Accuracy

| Condition              | Accuracy (FIXED) |
|------------------------|------------------|
| Short baseline (< 5 km) | 1–3 cm horizontal |
| Long baseline (30 km)  | 2–5 cm horizontal |
| Urban (multipath)      | 3–15 cm (frequent float) |
| High dynamics          | 2–8 cm           |

---

## 10. PPP — Precise Point Positioning

PPP uses a **single receiver** (no base station) with **precise satellite orbit and clock products** from an external service (IGS, CLAS, MADOCA). It requires carrier phase, dual-frequency, and typically 10–30 minutes to converge.

### 10.1 The PPP Observation Equations

Ionosphere-free combination (removes ionosphere to first order):

$$P_{IF} = C_1 P_1 + C_2 P_2 = \rho + c(\delta t_r - \delta t^s) + T + b_r^{IF} - b^{s,IF} + \varepsilon_P$$

$$\Phi_{IF} = C_1 \Phi_1 + C_2 \Phi_2 = \rho + c(\delta t_r - \delta t^s) + T + \lambda_{IF} N_{IF} + \phi_r^{IF} - \phi^{s,IF} + \varepsilon_\Phi$$

where $b$ and $\phi$ are hardware biases (DCBs) absorbed into the clock and ambiguity parameters.

### 10.2 PPP State Vector

Compared to SPP, PPP adds many more states:

$$\mathbf{x} = [\underbrace{x,y,z}_{\text{position}},\; \underbrace{c\,\delta t_r}_{\text{clock}},\; \underbrace{Z_T}_{\text{wet zenith troposphere}},\; \underbrace{N_{IF}^1, N_{IF}^2, \ldots}_{\text{IF ambiguities per satellite}}]$$

Typically 4 + 1 + n states where n = number of tracked satellites.

### 10.3 Precise Products

For PPP-level accuracy, broadcast ephemeris is insufficient. You need:

| Product          | Accuracy   | Source                         |
|------------------|------------|--------------------------------|
| Precise orbits   | ~2–5 cm    | IGS SP3 files                  |
| Precise clocks   | ~0.05 ns   | IGS CLK files                  |
| Satellite biases | ~0.1–1 ns  | Bias-SINEX / DCB files         |
| Ionosphere maps  | ~2–4 TECU  | IONEX files                    |
| RTCM SSR         | real-time  | NTRIP broadcast                |
| QZSS L6 (CLAS)   | ~4 cm      | L6 signal direct from satellite|

```cpp
// ppp.hpp
bool loadPreciseProducts(const std::string& orbit_file, const std::string& clock_file);
bool loadSSRProducts(const std::string& ssr_file);
bool loadL6Products(const std::string& l6_file);
bool loadIONEXProducts(const std::string& ionex_file);
bool loadDCBProducts(const std::string& dcb_file);
```

### 10.4 PPP-AR: Ambiguity Resolution

Standard PPP uses the ionosphere-free combination. Its ambiguity $N_{IF}$ is **not integer** due to hardware biases. PPP-AR (PPP with Ambiguity Resolution) restores integer-fixable ambiguities by applying satellite bias corrections:

- **IRC (Integer Recovery Clock)** / **UPD (Undifferenced Phase Bias)** corrections
- Wide-lane ambiguity fixed first (longer wavelength, easier)
- Narrow-lane fixed second

This brings PPP accuracy after convergence to ~2–5 cm, comparable to RTK without a base station.

### 10.5 PPP Convergence

PPP takes time to converge because:
1. Troposphere wet delay is estimated and must converge
2. Ambiguities must accumulate enough geometry diversity

Typical convergence time: 15–30 minutes for float PPP, 5–15 minutes for PPP-AR.

```cpp
bool hasConverged() const { return converged_; }
double getConvergenceTime() const { return convergence_time_; }
```

### 10.6 CLAS / MADOCA (State Space Representation)

Japan's QZSS satellite broadcasts **CLAS** (Centimetre Level Augmentation Service) on the L6 signal. This provides real-time corrections in **State Space Representation (SSR)** format: orbit corrections, clock corrections, phase biases, ionosphere/troposphere corrections. This allows PPP convergence in < 1 minute in Japan.

libgnss++ has a full CLAS/MADOCA decoder and PPP engine:
```cpp
// io/qzss_l6.hpp — L6 signal decoder
// algorithms/ppp_clas.hpp — CLAS-specific PPP processing
```

---

## 11. The Kalman Filter (State Estimation)

The Kalman filter is the mathematical engine behind RTK and PPP. It optimally combines a **prediction** (from a system model) with a **measurement** (from observations), accounting for noise in both.

### 11.1 Intuition

Imagine you are tracking a car:
- **Prediction:** "Based on its last position and velocity, it should be 10 m to the north now." (uncertain — you don't know exact speed)
- **Measurement:** "The GPS says it is 8 m to the north." (also uncertain — GPS noise)
- **Optimal estimate:** Weighted average, weighted by inverse variance. If GPS is more reliable, trust it more.

### 11.2 The Two Steps

**Predict (time update):**
$$\mathbf{x}_{k|k-1} = \mathbf{F}\,\mathbf{x}_{k-1|k-1}$$
$$\mathbf{P}_{k|k-1} = \mathbf{F}\,\mathbf{P}_{k-1|k-1}\,\mathbf{F}^\top + \mathbf{Q}$$

where **F** is the state transition matrix (how state evolves), **Q** is the process noise covariance (how uncertain the prediction is).

**Update (measurement update):**
$$\mathbf{v}_k = \mathbf{z}_k - \mathbf{H}\,\mathbf{x}_{k|k-1} \qquad \text{(innovation)}$$
$$\mathbf{S}_k = \mathbf{H}\,\mathbf{P}_{k|k-1}\,\mathbf{H}^\top + \mathbf{R} \qquad \text{(innovation covariance)}$$
$$\mathbf{K}_k = \mathbf{P}_{k|k-1}\,\mathbf{H}^\top\,\mathbf{S}_k^{-1} \qquad \text{(Kalman gain)}$$
$$\mathbf{x}_{k|k} = \mathbf{x}_{k|k-1} + \mathbf{K}_k\,\mathbf{v}_k$$
$$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k\,\mathbf{H})\,\mathbf{P}_{k|k-1}$$

where **H** is the observation matrix, **R** is the measurement noise covariance, and **K** is the Kalman gain (how much to trust the measurement).

### 11.3 RTKLIB-Compatible Implementation

libgnss++ uses the RTKLIB-compatible approach: only update "active" states (where the state is non-zero and variance is positive). This supports sparse-state estimation where ambiguities are initialized only when first observed.

```cpp
// kalman.hpp
// Collect active state indices: x[i] != 0.0 && P[i,i] > 0.0
std::vector<int> ix;
for (int i = 0; i < n; ++i) {
    if (x(i) != 0.0 && P(i, i) > 0.0) ix.push_back(i);
}
// Extract active sub-vectors/matrices
// ... solve reduced system ...
// Write back to full state
```

### 11.4 RTK Process Noise

For RTK, the position and ambiguity process noises reflect physical reality:
```cpp
double process_noise_position  = 1e-4;  // m²/s — small for near-static baseline
double process_noise_ambiguity = 1e-8;  // cycles²/s — very small (ambiguities are constant!)
```

### 11.5 PPP Process Noise

```cpp
// ppp_shared.hpp (typical values)
// Position: larger for kinematic mode
// Clock: large (receiver clock jumps unpredictably)
// Troposphere wet: moderate (slowly varying)
// Ambiguities: near-zero (constant between cycle slips)
```

---

## 12. Integer Ambiguity Resolution (LAMBDA)

The LAMBDA (Least-squares AMBiguity Decorrelation Adjustment) method is the gold standard for resolving carrier phase integer ambiguities.

### 12.1 The Problem

After the Kalman filter, you have:
- **Float ambiguities** $\hat{\mathbf{N}}$ (real-valued estimates, e.g., 10.23, -5.87, ...)
- **Ambiguity covariance matrix** $\mathbf{Q}_N$ (their uncertainties)

You need to find the integer vector $\mathbf{N}$ that best fits:

$$\hat{\mathbf{N}} = \arg\min_{\mathbf{N}\in\mathbb{Z}^n} (\hat{\mathbf{N}} - \mathbf{N})^\top \mathbf{Q}_N^{-1} (\hat{\mathbf{N}} - \mathbf{N})$$

This is an **Integer Least Squares (ILS)** problem — NP-hard in general, but the LAMBDA decorrelation makes it tractable.

### 12.2 Decorrelation

The ambiguity covariance matrix is often highly correlated (because you form differences). Highly correlated unknowns make the search space very elongated and slow.

LAMBDA applies a sequence of **unimodular integer transformations** $\mathbf{Z}$ to decorrelate:

$$\mathbf{Q}_z = \mathbf{Z}^\top \mathbf{Q}_N \mathbf{Z}$$

After transformation the search ellipsoid is more spherical → much faster search.

### 12.3 Modified LAMBDA (MLAMBDA) Search

After decorrelation, find the two best integer candidates in the reduced space using a branch-and-bound search. Transform back to the original ambiguity space.

```cpp
// lambda.hpp
/*
 * [1] P.J.G.Teunissen, The least-square ambiguity decorrelation adjustment,
 *     J.Geodesy, Vol.70, 65-82, 1995
 * [2] X.-W.Chang, X.Yang, T.Zhou, MLAMBDA: A modified LAMBDA method,
 *     J.Geodesy, Vol.79, 552-565, 2005
 */
bool lambdaSearch(const VectorXd& float_amb, const MatrixXd& Q_amb,
                  VectorXd& fixed_amb, double& ratio);
```

### 12.4 Ratio Test

The ratio test validates the fix:

$$r = \frac{\|\hat{\mathbf{N}} - \mathbf{N}_2\|^2_{\mathbf{Q}_N^{-1}}}{\|\hat{\mathbf{N}} - \mathbf{N}_1\|^2_{\mathbf{Q}_N^{-1}}}$$

where $\mathbf{N}_1$ is the best and $\mathbf{N}_2$ is the second-best integer candidate.

If $r > r_{threshold}$ (usually 2.5–3.0), the best candidate is clearly better than the second-best → accept the fix. Otherwise, stay with float solution.

```cpp
double ratio_threshold = 3.0;    // from rtk.hpp
double& ratio = solution.ratio;  // reported in solution
```

**Wide-lane first, narrow-lane second:** For L1/L2 RTK, first fix the wide-lane ambiguity ($\lambda_{WL} \approx 86$ cm, easy to fix), then use it to fix the narrow-lane ($\lambda_{NL} \approx 10.7$ cm).

---

## 13. Quality Indicators and Validation

### 13.1 DOP — Dilution of Precision

DOP is a scalar measure of satellite geometry quality. It tells you: "given the current satellite positions, how much does geometry *amplify* the measurement noise?"

$$\mathbf{Q} = (\mathbf{H}^\top\mathbf{H})^{-1}$$

| DOP   | Formula                                  | Meaning                   |
|-------|------------------------------------------|---------------------------|
| PDOP  | $\sqrt{Q_{11}+Q_{22}+Q_{33}}$           | 3D position               |
| HDOP  | $\sqrt{Q_{11}+Q_{22}}$ (in ENU)          | Horizontal                |
| VDOP  | $\sqrt{Q_{33}}$ (in ENU)                 | Vertical                  |
| TDOP  | $\sqrt{Q_{44}}/c$                        | Time / clock              |
| GDOP  | $\sqrt{Q_{11}+Q_{22}+Q_{33}+Q_{44}/c^2}$| Geometric (all)           |

Typical good PDOP: < 2.0. PDOP > 6 means degraded geometry, expect worse accuracy.

```cpp
struct DOPValues {
    double gdop = 999.9;
    double pdop = 999.9;
    double hdop = 999.9;
    double vdop = 999.9;
    double tdop = 999.9;
};
```

### 13.2 Solution Status

libgnss++ reports the solution status at every epoch:

```cpp
enum class SolutionStatus {
    NONE,       // no solution (< 4 satellites, or singular geometry)
    SPP,        // standalone pseudorange
    DGPS,       // differential GPS
    FLOAT,      // RTK float (ambiguities not fixed)
    FIXED,      // RTK fixed (integers resolved)
    PPP_FLOAT,  // PPP converging
    PPP_FIXED   // PPP with ambiguity resolution
};
```

### 13.3 Position Covariance and 2D/3D Accuracy

From the least-squares solution covariance:
$$\sigma_{2D} = \sqrt{P_{EE} + P_{NN}}, \quad \sigma_{3D} = \sqrt{P_{EE}+P_{NN}+P_{UU}}$$

The `PositionSolution` struct provides:
```cpp
double getHorizontalAccuracy() const;  // 95% confidence ellipse semi-major axis
double getVerticalAccuracy() const;
double get3DAccuracy() const;
```

### 13.4 Residuals

After solving, the *residual* for each satellite is:

$$v_i = P_i^{meas} - P_i^{predicted}$$

Large residuals (> 3σ) indicate multipath, cycle slips, or bad measurements. libgnss++ reports per-satellite residuals:
```cpp
std::vector<double> satellite_residuals;
double residual_rms;
```

### 13.5 PDOP-Based Data Quality Control

Epochs with PDOP > 30 are typically rejected as unreliable. This prevents bad geometry from corrupting RTK/PPP filter states.

---

## 14. Data Formats

### 14.1 RINEX — Receiver Independent Exchange Format

The universal standard for GNSS data exchange.

#### RINEX Observation File (.obs / .rnx)
Contains epoch-by-epoch measurement data:
```
> 2023  3  1  0  0  0.0000000  0  8
G01  23456789.123   23456789.234    45.0
G08  21234567.456   21234567.567    42.3
...
```
Each line: satellite ID, pseudorange (m), carrier phase (cycles), SNR, Doppler.

#### RINEX Navigation File (.nav / .rnx)
Contains satellite ephemeris data. For each satellite, one record with all the orbital parameters (sqrt_a, e, i0, omega0, ...) every 2 hours.

libgnss++ reads both in `io/rinex.hpp`.

### 14.2 RTCM — Radio Technical Commission for Maritime

Binary format for real-time corrections. Used for RTK base station corrections over Internet (NTRIP protocol) or radio link.

Key message types:
| Type | Content                                     |
|------|---------------------------------------------|
| 1001–1004 | GPS RTK observations (pseudorange + phase) |
| 1005–1006 | Reference station coordinates              |
| 1019 | GPS ephemeris                               |
| 1074–1077 | GPS MSM (Modern Signal Messages)          |
| 1085 | GLONASS MSM                                |
| 1094 | Galileo MSM                                |
| 1074–1077 | GPS satellite-based SSR corrections      |
| 1059 | GPS code bias                              |
| 1265 | Phase bias (for PPP-AR)                    |

```cpp
// io/rtcm.hpp — RTCM parser
// io/rtcm_stream.hpp — streaming RTCM with NTRIP
```

### 14.3 SP3 — Standard Product 3

Precise orbit files from IGS. Provides satellite ECEF positions and clock values at 5- or 15-minute intervals (interpolated for intermediate times using polynomial or Lagrange interpolation).

### 14.4 CLK — RINEX Clock File

Precise satellite and receiver clock offsets from IGS, at 30-second or 5-second intervals.

### 14.5 UBX — u-blox Binary Protocol

Proprietary binary format from u-blox receiver chips. Provides raw measurements (pseudorange, carrier phase, Doppler) in a compact binary format:
```cpp
// io/ubx.hpp — UBX protocol parser
```

### 14.6 IONEX — IONosphere map EXchange

IONEX files contain global TEC maps at 2-hour intervals on a geographic grid (5° × 2.5° typical). Used in PPP for ionosphere correction or GNSS integrity monitoring.

### 14.7 SBF — Septentrio Binary Format

Binary format from Septentrio receivers:
```cpp
// io/sbf.hpp
```

### 14.8 NMEA — National Marine Electronics Association

The output format that GNSS receivers use to communicate position to applications. Key sentences:
- `$GPGGA` — fix data (lat/lon/height/satellites/HDOP)
- `$GPRMC` — minimum recommended data (time/lat/lon/velocity/date)

```cpp
std::string toNMEA() const;  // in solution.hpp
```

---

## 15. libgnss++ Architecture Walkthrough

### 15.1 Overall Layer Model

```
┌─────────────────────────────────────────────────────┐
│                    Applications                     │
│  CLI tools, Python bindings, ROS2 nodes, Web UI     │
├─────────────────────────────────────────────────────┤
│                    Algorithms                       │
│  SPPProcessor  RTKProcessor  PPPProcessor           │
│  kalmanFilter  lambdaSearch                         │
│  rtk_selection rtk_measurement rtk_update           │
├─────────────────────────────────────────────────────┤
│                    Models                           │
│  tropDelaySaastamoinen  ionoDelayKlobuchar          │
│  niellHydrostaticMapping  niellWetMapping           │
├─────────────────────────────────────────────────────┤
│                    Core                             │
│  constants  types  coordinates  navigation          │
│  observation  solution  processor  signal_policy    │
├─────────────────────────────────────────────────────┤
│                    I/O                              │
│  rinex  rtcm  ubx  ntrip  sbf  qzss_l6              │
│  solution_writer  serial_port                       │
└─────────────────────────────────────────────────────┘
```

### 15.2 Core Module (`include/libgnss++/core/`)

| File            | What it contains                                |
|-----------------|--------------------------------------------------|
| `constants.hpp` | All physical constants, frequencies, WGS-84 params |
| `types.hpp`     | `GNSSSystem`, `SignalType`, `GNSSTime`, `SolutionStatus` |
| `coordinates.hpp` | `ecef2geodetic`, `geodetic2ecef`, `ecef2enu`, `geodist` |
| `observation.hpp` | `Observation` struct, `ObservationData` epoch container |
| `navigation.hpp` | `Ephemeris`, `NavigationData`, `IonosphereModel`, `TroposphereModel` |
| `solution.hpp`  | `PositionSolution`, `Solution` container, statistics |
| `processor.hpp` | `ProcessorBase` interface, `ProcessorConfig`, `ProcessorStats` |
| `signal_policy.hpp` | Shared signal selection rules for all processors |

### 15.3 How to Process a RINEX File (Code Walkthrough)

```cpp
#include <libgnss++/gnss.hpp>

// 1. Create processor
libgnss::SPPProcessor spp;
libgnss::ProcessorConfig config;
config.elevation_mask = 15.0;
config.use_ionosphere_model = true;
config.use_troposphere_model = true;
spp.initialize(config);

// 2. Load data (via GNSSProcessor wrapper or directly via RINEXReader)
libgnss::GNSSProcessor gnss;
gnss.setMode(libgnss::GNSSProcessor::Mode::SPP);
auto solution = gnss.processFile("obs.rnx", "nav.rnx");

// 3. Iterate over solutions
for (const auto& sol : solution.solutions) {
    auto& geo = sol.position_geodetic;
    printf("Time: %d %.3f  Lat: %.8f  Lon: %.8f  H: %.3f  PDOP: %.2f  Sats: %d\n",
           sol.time.week, sol.time.tow,
           geo.latitude * 180/M_PI,
           geo.longitude * 180/M_PI,
           geo.height,
           sol.pdop, sol.num_satellites);
}
```

### 15.4 SPPProcessor Internal Flow

```
SPPProcessor::processEpoch(obs, nav)
│
├─ validateObservations(obs, nav, time)
│    For each obs:
│    ├─ check isPrimarySPPSignal (L1CA / E1 / B1I / ...)
│    ├─ check obs.valid && has_pseudorange && pseudorange > 0
│    ├─ skip BeiDou GEO satellites
│    └─ check nav.hasEphemeris at transmission time
│
└─ solvePositionLS(valid_obs, nav, time)
     │
     ├─ ITER: buildMeasurements(position)
     │    For each obs:
     │    ├─ compute transmission time: tx = t - P/c
     │    ├─ nav.calculateSatelliteState(sat, tx) → sat_pos, sat_clk
     │    ├─ correct sat_clk → refine tx → recompute sat_pos
     │    ├─ apply Sagnac rotation to sat_pos
     │    ├─ nav.calculateGeometry() → elevation, azimuth
     │    ├─ skip if elevation < 5°
     │    ├─ tropDelaySaastamoinen(rcv_pos, el) → trp
     │    ├─ ionoDelayKlobuchar(lat,lon,az,el,tow,α,β) → ion
     │    ├─ groupDelayCorrectionMeters(obs, eph) → tgd
     │    └─ corrected_PR = P + sat_clk·c - trp - ion - tgd
     │
     ├─ build H matrix (line-of-sight vectors, clock column)
     ├─ build weight matrix (sin²(elevation))
     ├─ WLS solve: dx = QR(weighted_H) \ weighted_residuals
     ├─ update: position += dx[0:3], clock += dx[3]
     └─ if ||dx[0:3]|| < 0.1 mm → converged
```

### 15.5 RTKProcessor Internal Flow

```
RTKProcessor::processEpoch(rover_obs, nav)  + base position/obs
│
├─ SPP fallback for initial position
├─ rtk_selection: select reference satellite, build DD pairs
├─ Kalman predict: add process noise to P
├─ rtk_measurement: build DD residuals, DD covariance
│   ├─ for each DD pair: compute predicted DD phase & code
│   └─ assemble H (DD observation matrix, maps SD states to DD obs)
├─ rtk_update: kalmanFilter(x, P, H, v, R)
├─ rtk_slip_detection: check for cycle slips → reset ambiguities
└─ ambiguity resolution attempt:
    ├─ rtk_ar_selection: select subset of ambiguities
    ├─ lambdaSearch(N_float, Q_N) → N_fixed, ratio
    ├─ rtk_validation: ratio test, position jump check
    └─ if valid: report FIXED, else report FLOAT
```

### 15.6 File Structure Summary

```
gnssplusplus-library/
├── include/libgnss++/
│   ├── gnss.hpp              ← main include (includes everything)
│   ├── core/
│   │   ├── constants.hpp     ← physical constants
│   │   ├── types.hpp         ← enums, GNSSTime, SatelliteId
│   │   ├── coordinates.hpp   ← ECEF/geodetic/ENU conversions
│   │   ├── observation.hpp   ← Observation, ObservationData
│   │   ├── navigation.hpp    ← Ephemeris, NavigationData
│   │   ├── solution.hpp      ← PositionSolution, Solution
│   │   └── processor.hpp     ← ProcessorBase, ProcessorConfig
│   ├── algorithms/
│   │   ├── spp.hpp           ← SPPProcessor
│   │   ├── rtk.hpp           ← RTKProcessor
│   │   ├── ppp.hpp           ← PPPProcessor
│   │   ├── kalman.hpp        ← Kalman filter update
│   │   └── lambda.hpp        ← LAMBDA ambiguity resolution
│   ├── models/
│   │   ├── troposphere.hpp   ← Saastamoinen, Niell MF
│   │   └── ionosphere.hpp    ← Klobuchar model
│   └── io/
│       ├── rinex.hpp         ← RINEX 2/3 obs+nav reader
│       ├── rtcm.hpp          ← RTCM 2/3 parser
│       ├── ubx.hpp           ← u-blox UBX parser
│       ├── ntrip.hpp         ← NTRIP client
│       └── solution_writer.hpp ← write pos/NMEA/KML output
└── src/
    ├── algorithms/
    │   ├── spp.cpp           ← SPP implementation
    │   ├── rtk.cpp           ← RTK implementation
    │   └── ppp.cpp           ← PPP implementation
    └── core/
        └── navigation.cpp    ← ephemeris propagation, RINEX/SP3/CLK parsing
```

---

## 16. Summary: The Three Positioning Modes

### 16.1 Side-by-Side Comparison

| Feature                    | SPP                 | RTK                    | PPP                      |
|----------------------------|---------------------|------------------------|--------------------------|
| Measurement type           | Pseudorange         | Carrier phase (+ code) | Carrier phase + code     |
| Infrastructure needed      | None                | Base station           | Precise products (IGS/CLAS) |
| Typical accuracy           | 1–5 m               | 1–5 cm (fixed)         | 2–10 cm (after converge) |
| Convergence time           | Immediate           | 10–60 s (after fix)    | 10–30 min (float PPP)    |
| Integer ambiguities        | N/A                 | DD ambiguities         | IF or undifferenced       |
| Baseline limit             | N/A                 | < 50 km                | Global                   |
| GNSS systems               | All                 | All                    | All                      |
| Key corrections            | Klobuchar + Saastamoinen | DD cancellation   | Precise orbits/clocks, Niell, IONEX |
| Ionosphere handling        | Klobuchar model     | DD cancels (short base)| IF combination or IONEX   |
| Troposphere handling       | Saastamoinen        | DD ≈ cancels (short base) | Saastamoinen + ZTD estimate |
| libgnss++ class            | `SPPProcessor`      | `RTKProcessor`         | `PPPProcessor`           |
| Suitable use case          | Navigation, fleet   | Surveying, autonomous vehicles | Remote areas, precision agriculture |

### 16.2 Math Complexity Summary

| Concept                   | Used in          | Section          |
|---------------------------|------------------|------------------|
| 3D distance formula       | SPP, RTK, PPP    | 0.3, 8.2         |
| Unit vectors / dot product | All             | 0.4, 8.2         |
| Matrix multiply + inverse  | All             | 0.5, 0.6         |
| Weighted least squares     | SPP, RTK, PPP    | 0.6, 8.2         |
| Iterative Newton method    | SPP, ephemeris   | 0.8, 6.2, 8.2    |
| Taylor linearization       | SPP, RTK, PPP    | 0.9, 8.2         |
| Trigonometry (sin/cos)     | All             | 0.2, 3.1, 6.2    |
| Gaussian statistics / RMS  | All             | 0.7              |
| Kalman filter              | RTK, PPP         | 11               |
| Integer least squares      | RTK, PPP-AR      | 12               |

### 16.3 Learning Path Recommendation

```
Week 1–2:  Math 0.1–0.5 (notation, trig, vectors, matrices)
Week 3:    Math 0.6–0.9 (least squares, statistics, iteration, linearization)
Week 4:    GNSS 1–5 (big picture, signals, coordinates, time, observations)
Week 5:    GNSS 6–7 (ephemeris computation, error sources)
Week 6:    GNSS 8 (SPP — read spp.cpp in detail, trace through the code)
Week 7:    GNSS 11 (Kalman filter — read kalman.hpp, understand RTK's use)
Week 8:    GNSS 9 (RTK — read rtk.hpp, rtk_measurement.hpp, rtk_update.hpp)
Week 9:    GNSS 12 (LAMBDA — read lambda.hpp, understand ratio test)
Week 10+:  GNSS 10 (PPP — builds on all prior knowledge)
```

### 16.4 Debugging Checklist for New Developers

When the position looks wrong or the fix rate is poor:

1. **Check satellite count:** `solution.num_satellites < 6` → poor geometry
2. **Check PDOP:** `solution.pdop > 4.0` → spread satellites are better
3. **Check residuals:** `solution.residual_rms > 5 m` → outliers, multipath, wrong model
4. **Check elevation mask:** raising to 20° reduces multipath but loses satellites
5. **Check ephemeris validity:** `nav.getEphemeris(sat, time)` returns nullptr → no nav data for that satellite
6. **Check time:** wrong GPS week rollover causes many satellites to fail ephemeris lookup
7. **For RTK:** check baseline length, base station coordinates, and signal availability on both base and rover
8. **For PPP:** wait for convergence (15–30 min), check precise product time coverage

---

## Appendix A: Physical Constants Reference

All from `constants.hpp`:

| Constant     | Value                        | Unit    | Description                    |
|--------------|------------------------------|---------|--------------------------------|
| c            | 299,792,458.0                | m/s     | Speed of light                  |
| Ω_E          | 7.2921151467 × 10⁻⁵          | rad/s   | Earth rotation rate             |
| μ_GPS        | 3.986005 × 10¹⁴              | m³/s²   | Earth gravitational constant    |
| F_rel        | −4.442807633 × 10⁻¹⁰         | s/√m    | Relativistic correction factor  |
| a_WGS84      | 6,378,137.0                  | m       | Semi-major axis                 |
| f_WGS84      | 1/298.257223563              | —       | Flattening                      |
| e²_WGS84     | 6.694379990 × 10⁻³           | —       | First eccentricity squared      |
| GPS_L1_freq  | 1,575.42 × 10⁶               | Hz      | GPS L1 frequency                |
| GPS_L1_λ     | 0.19029 m                    | m       | GPS L1 wavelength               |
| GPS_WL_λ     | ≈ 0.8619 m                   | m       | GPS wide-lane wavelength        |
| GPS_NL_λ     | ≈ 0.1070 m                   | m       | GPS narrow-lane wavelength      |

---

## Appendix B: Glossary

| Term            | Full form / explanation                                          |
|-----------------|------------------------------------------------------------------|
| CLAS            | Centimetre Level Augmentation Service (QZSS Japan)              |
| DCB             | Differential Code Bias                                           |
| DD              | Double Difference                                                |
| DOP             | Dilution of Precision                                            |
| ECEF            | Earth-Centred Earth-Fixed                                        |
| ENU             | East-North-Up (local coordinate frame)                          |
| GEO             | Geostationary Earth Orbit                                        |
| GNSS            | Global Navigation Satellite System                               |
| IGS             | International GNSS Service                                       |
| IF              | Ionosphere-Free (linear combination)                             |
| IONEX           | IONosphere map EXchange format                                   |
| ILS             | Integer Least-Squares                                            |
| LAMBDA          | Least-squares AMBiguity Decorrelation Adjustment                 |
| LOS             | Line of Sight                                                    |
| MADOCA          | Multi-GNSS Advanced Demonstration Tool for Orbit and Clock Analysis |
| MEO             | Medium Earth Orbit                                               |
| NMF             | Niell Mapping Function                                           |
| NL              | Narrow-Lane                                                      |
| NTRIP           | Networked Transport of RTCM via Internet Protocol               |
| PCO/PCV         | Phase Centre Offset / Phase Centre Variation                     |
| PPP             | Precise Point Positioning                                        |
| PPP-AR          | PPP with Ambiguity Resolution                                    |
| PRN             | Pseudo-Random Noise (code)                                       |
| RTCM            | Radio Technical Commission for Maritime (binary format)         |
| RTK             | Real-Time Kinematic                                              |
| SD              | Single Difference                                                |
| SNR             | Signal-to-Noise Ratio                                            |
| SP3             | Standard Product 3 (precise orbit file format)                  |
| SPP             | Single Point Positioning                                         |
| SSR             | State Space Representation                                       |
| TEC             | Total Electron Content                                           |
| TECU            | TEC Units (10¹⁶ electrons/m²)                                   |
| TGD             | Time Group Delay (hardware bias)                                 |
| TOW             | Time of Week                                                     |
| UPD             | Undifferenced Phase Biases                                       |
| WGS-84          | World Geodetic System 1984                                       |
| WL              | Wide-Lane                                                        |
| ZTD             | Zenith Total Delay (troposphere)                                 |

---

*This guide is grounded in the libgnss++ source code at [github.com/rsasaki0109/gnssplusplus-library](https://github.com/rsasaki0109/gnssplusplus-library). All code snippets are taken directly from the library headers and source files.*

*Reference docs: https://rsasaki0109.github.io/gnssplusplus-library/*