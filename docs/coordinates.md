# Coordinate System

## Introduction

A mm antenna mounted on drones has the possibility to be used as alternatives to other radio based communication such as GPS when those signals are jammed. In respective to this, we want our hackathon to be based on only the relative positions of each drones using vectors.

For the following sections we will discuss how we get a reference for each, how the drone is able to relate to every vector, and how we get the target area.

## The drones

We will combine the first two questions into one as they are both managed by the drones.

What we need for each and every one of our drones is a basis vector so that we can use the information given from other drones, otherwise tasks such as Assisted conical scan or [Spiral search](seeking.md) simply won't work. Luckily for us we can simply use an accelerometer to use gravity and the front of the drone as our basis vectors. We will only represent gravity in our graphs to minimize clutter. It will be a dotted vector shown below:

```tikz
\usetikzlibrary{arrows.meta,calc}
\begin{document}

% ---- Catppuccin Latte ----
\definecolor{ctpBase}    {HTML}{EFF1F5}
\definecolor{ctpMantle}  {HTML}{E6E9EF}
\definecolor{ctpSurface1}{HTML}{BCC0CC}
\definecolor{ctpOverlay1}{HTML}{8C8FA1}
\definecolor{ctpOverlay0}{HTML}{9CA0B0}
\definecolor{ctpText}    {HTML}{4C4F69}
\definecolor{ctpRed}     {HTML}{D20F39}
\definecolor{ctpBlue}    {HTML}{1E66F5}
\definecolor{ctpFlamingo}{HTML}{DD7878}
\definecolor{ctpLavender}{HTML}{7287FD}

% ---- the two points: change only these ----
\def\Px{3} \def\Py{3} \def\Pz{2}

\begin{tikzpicture}[
  x={(1cm,0cm)}, y={(0.4cm,0.2cm)}, z={(0cm,1cm)},
  text=ctpText,
  vec/.style={-{Stealth[length=3mm]}, very thick, shorten >=2pt},
  drop/.style={dashed, ctpOverlay0, thin},
  foot/.style={circle, fill=ctpOverlay1, inner sep=1.1pt},
  dot/.style={circle, fill=ctpText, inner sep=1.5pt}
]

% ground plane + grid
\fill[ctpMantle] (0,0,0) -- (7,0,0) -- (7,6,0) -- (0,6,0) -- cycle;
\foreach \i in {0,...,7} \draw[ctpSurface1, very thin] (\i,0,0) -- (\i,6,0);
\foreach \j in {0,...,6} \draw[ctpSurface1, very thin] (0,\j,0) -- (7,\j,0);

% axes
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (7.8,0,0) node[below right] {$x$};
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (0,6.8,0) node[right] {$y$};
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (0,0,4.4) node[above] {$z$};

% the two points, their feet, and the derived middle point
\coordinate (P)  at (\Px,\Py,\Pz);

% Vectors
\draw[vec, ctpOverlay0, dotted] (P) -- ++(0,0,-1.25);

\node[dot] at (P) {}; \node[left]  at (P) {$P$};
\end{tikzpicture}
\end{document}
```

Now in order for us to be able to refer to other points we will simply use the *Elevation* and *Azimuth* to represent each point. Since each drone has 2-3 antennas our graphs will look like:

```tikz
\usetikzlibrary{arrows.meta,calc}
\begin{document}

% ---- Catppuccin Latte ----
\definecolor{ctpBase}    {HTML}{EFF1F5}
\definecolor{ctpMantle}  {HTML}{E6E9EF}
\definecolor{ctpSurface1}{HTML}{BCC0CC}
\definecolor{ctpOverlay1}{HTML}{8C8FA1}
\definecolor{ctpOverlay0}{HTML}{9CA0B0}
\definecolor{ctpText}    {HTML}{4C4F69}
\definecolor{ctpRed}     {HTML}{D20F39}
\definecolor{ctpBlue}    {HTML}{1E66F5}
\definecolor{ctpFlamingo}{HTML}{DD7878}
\definecolor{ctpLavender}{HTML}{7287FD}

\begin{tikzpicture}[
  x={(1cm,0cm)}, y={(0.4cm,0.2cm)}, z={(0cm,1cm)},
  text=ctpText,
  vec/.style={-{Stealth[length=3mm]}, very thick, shorten >=2pt},
  drop/.style={dashed, ctpOverlay0, thin},
  foot/.style={circle, fill=ctpOverlay1, inner sep=1.1pt},
  dot/.style={circle, fill=ctpText, inner sep=1.5pt}
]

% ground plane + grid
\fill[ctpMantle] (0,0,0) -- (7,0,0) -- (7,6,0) -- (0,6,0) -- cycle;
\foreach \i in {0,...,7} \draw[ctpSurface1, very thin] (\i,0,0) -- (\i,6,0);
\foreach \j in {0,...,6} \draw[ctpSurface1, very thin] (0,\j,0) -- (7,\j,0);

% axes
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (7.8,0,0) node[below right] {$x$};
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (0,6.8,0) node[right] {$y$};
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (0,0,4.4) node[above] {$z$};

% the two points, their feet, and the derived middle point
\coordinate (P)  at (3,3,2);
\coordinate (Q)  at (6,2,3);
\coordinate (Q2) at (3,1,1);

% Vectors
\draw[vec, ctpRed] (P) -- (Q) node[midway, above left]  {$\vec{u}$};
\draw[vec, ctpFlamingo] (P) -- (Q2) node[midway, above left]  {$\vec{v}$};
\draw[vec, ctpOverlay0, dotted] (P) -- ++(0,0,-1.25);

\node[dot] at (P) {}; \node[left]  at (P) {$P$};
\end{tikzpicture}
\end{document}
```

Now when we have the angles and a basis vector. How do we determine the length of each vector? This question is particularly tricky as we can not guarantee time-sync (unless every drone has an atomic clock) without gps. Our solution to this is still to use time, but instead make each drone return the signal as soon as they receive it. Therefore the time can be calculated simply as the timestamp difference. That way we can calculate the length of our vector as:
$$
L = (t - t_d) \times c \times \frac 1 2
$$
where
$$
\begin{eqnarray}
L &&: \text{Length in m} \\
t &&: \text{Time stamp difference} \\
t_{d} &&: \text{offset for drone transfering the signal} \\
c &&: \text{speed of light} \\
\end{eqnarray}
$$

With this in mind we can simply just refer to every drone, item of interest as a direction from our drone. given that the [lookup table](communication.md) has the angles of the drone according to it's relation, we can simply just update the table so all points are in relation to us. This information can simply just be updated as we fly since we will always have a connection to another drone due to our conical scan algorithm

## The target area

The trickiest part of this is that we still need to respect that we have a target area which we are tasked to provide communication for. To solve this we can not have a system which can only relate to itself. We need some base which has some coordinates which the drones can then use to stay within the target area. Luckily we need to use bases to manage our swarm system anyway. So it can simply just transmit the points as vectors to the network which the drones can use for its swarm agent control.
Simply visualised:

```tikz
\usetikzlibrary{arrows.meta,calc}
\begin{document}

% ---- Catppuccin Latte ----
\definecolor{ctpBase}    {HTML}{EFF1F5}
\definecolor{ctpMantle}  {HTML}{E6E9EF}
\definecolor{ctpSurface1}{HTML}{BCC0CC}
\definecolor{ctpOverlay1}{HTML}{8C8FA1}
\definecolor{ctpOverlay0}{HTML}{9CA0B0}
\definecolor{ctpText}    {HTML}{4C4F69}
\definecolor{ctpRed}     {HTML}{D20F39}
\definecolor{ctpBlue}    {HTML}{1E66F5}
\definecolor{ctpFlamingo}{HTML}{DD7878}
\definecolor{ctpLavender}{HTML}{7287FD}

\begin{tikzpicture}[
  x={(1cm,0cm)}, y={(0.4cm,0.2cm)}, z={(0cm,1cm)},
  text=ctpText,
  vec/.style={-{Stealth[length=3mm]}, very thick, shorten >=2pt},
  drop/.style={dashed, ctpOverlay0, thin},
  foot/.style={circle, fill=ctpOverlay1, inner sep=1.1pt},
  dot/.style={circle, fill=ctpText, inner sep=1.5pt}
]

% ground plane + grid
\fill[ctpMantle] (0,0,0) -- (7,0,0) -- (7,6,0) -- (0,6,0) -- cycle;
\foreach \i in {0,...,7} \draw[ctpSurface1, very thin] (\i,0,0) -- (\i,6,0);
\foreach \j in {0,...,6} \draw[ctpSurface1, very thin] (0,\j,0) -- (7,\j,0);

% axes
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (7.8,0,0) node[below right] {$x$};
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (0,6.8,0) node[right] {$y$};
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (0,0,4.4) node[above] {$z$};

% the two points, their feet, and the derived middle point
\coordinate (P)  at (5,5,1);
\coordinate (B)  at (1,1,0);
\coordinate (C1)  at (1,6,0);
\coordinate (C2)  at (3,2,0);
\coordinate (C3)  at (7,1,0);

% Vectors
\draw[vec, ctpRed]  ($(P)+(0,0,0.05)$) -- ($(B)+(0,0,0.05)$);
\draw[vec, ctpBlue]  ($(B)+(0,0,-0.05)$) -- ($(P)+(0,0,-0.05)$);

\draw[vec, ctpOverlay0, dotted] (P) -- ++(0,0,-1.25);
\draw[vec, ctpLavender, dotted] (B) -- (C1);
\draw[vec, ctpLavender, dotted] (B) -- (C2);
\draw[vec, ctpLavender, dotted] (B) -- (C3);
\draw[vec, ctpFlamingo, dotted] (P) -- (C1);
\draw[vec, ctpFlamingo, dotted] (P) -- (C2);
\draw[vec, ctpFlamingo, dotted] (P) -- (C3);
\fill[ctpBlue, opacity=0.2] (C1)--(C2)--(C3)--cycle;
% Filled triangular prism height = 3

% bottom face
\fill[ctpBlue, opacity=0.15] (C1)--(C2)--(C3)--cycle;

\node[dot] at (P) {}; \node[right]  at (P) {$P$};
\node[dot] at (B) {}; \node[left]  at (B) {$B$};
\end{tikzpicture}
\end{document}
```

> Note that the target area is the filled area derived from the dotted vectors.

The drone should be able to very easily derive if its within the area or not based on this information shared by the communication from the base as shown in the graph.
