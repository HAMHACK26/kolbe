# Communication

## Introduction

This is a collection of notes on the communication which we will use for swarm management. Note that, for a mm antenna we can assume we have a high bandwidth so these notes should not affect the communication of our swarm.

> Disclaimer: For our hackathon we used a simple int as the ID, but to describe the headers more effectively we decided to use uuid for this section.

Note that each drone broadcasts every 0.1 seconds (see [update rate](#update-rate)), sending the header, lookup table and coverage area, each of which we go over in its own section.

## Headers

Our headers are sent with every request and are structured as follows:

| id   | connected antenna | flight direction | time received |
| ---- | ----------------- | ---------------- | ------------- |
| UUID | vector            | vector           | datetime      |

The connected antenna is the exchange originator's local direction toward its
direct neighbour. When the request returns as an echo, the originator is the
receiver and uses this direction to calculate the neighbour vector in its
lookup table.

We use the connected antenna vector as a reference to see what the drone considers to be its front. This + the gravity vector (as mentioned in our [coordinates notes](coordinates.md)) allows us to extrapolate the vectors for all other drones, more on that later.

We use the flight direction for our assisted conical scan and the time received to calculate the length of the vector as mentioned in our [coordinates notes](coordinates.md).

## Lookup Table

The core part of our communication is the lookup table, which makes up the majority of the body for our network, each drone has their own lookup table with a vector that points to a drone in question **expressed in the base reference frame** (see base), which is updated in real time. Each item in this table is structured as follows:

| id   | timestamp | location | neighbour distance | connections |
| ---- | --------- | -------- | ------------------ | ----------- |
| uuid | datetime  | vector   | int                | list[uuid]  |

Because every `location` is stored in the base reference frame rather than relative to the sending drone, a receiving drone does **not** need to re-rotate each row through a chain of pairwise headings. Since each drone continuously knows its own vector to the base (the base is itself a node in the table) plus the shared gravity vector, it can project any local antenna reading into the base frame in real time and store it directly. This is what stops yaw error from accumulating hop by hop: every drone measures against the **same** anchor instead of chaining relative angles. The *connected antenna* vector in the header is still required to perform that one projection into the base frame for a direct neighbour.

In order to be able to bias data we receive we also increment the value *neighbour distance*, if it's 0 it indicates that the drone that transmitted it has a direct connection. Note that even in the base frame we keep this field: it no longer needs to correct yaw drift (the base frame already handles direction), but it still weights the confidence of the vector *length* (see below), drives the loop-closure weighting and the update-timeout logic.

When a drone receives a table from another drone it goes over each item row by row.
For storage, our id is the key in a key value store dataset. The data is then filtered by neighbour distance, as a closer neighbour distance would mean the reading is more accurate (due to our translation of the vector).
Obviously we do not disregard those that have a higher neighbour number, but instead our drone would expect an update in $u_t \times (n +1)$ seconds (where n is the neighbour distance and $u_t$ is the update time, which is 0.1 seconds in our case, more on that later). If an update is not received within this interval, then we can only assume that the drone which had that original connection has reconnected with another drone and we will override this row with this new data point.

Note that there is still a lot of drift when we do this calculation, hence why we have a discrimination to the closest data. We also want to note that this is the reason we need to use [spiral search](seeking.md) to actually make new connections.

We also have the connections and timestamp field which is mainly used for reconnection of drones, a timestamp will tell us how recently the data was added this will effectively indicate if this drone is currently out of commission as it needs to be connected to the network with at least one antenna for us to get an accurate reading of its current position.
The connections tab is similarly used to retain connection but it is actually used so we do not remove a connection to a drone if it has only one connection left, that connection being the drone with that particular id. We will also use this to recursively verify that the connection we cut isn't part of a longer chain which has no other connection to the ground base. Such as the example illustrated below.

### Timestamp provenance

Lookup timestamps are stored in the table owner's local clock domain. When a
row is relayed, its age is calculated on the sender's clock and then rebased
onto the receiver's clock:

$$
a = t_{sender} - t_{row}, \qquad t'_{row} = t_{receiver} - a
$$

This preserves row age without requiring synchronized drone clocks. Negative
or non-finite ages are invalid and must not replace the last valid row.

### Directed yaw observations

The UUID list in `connections` remains the lookup-table topology. Each direct
connection also produces internal directed edge metadata which is relayed with
the lookup table:

| from | to | measured yaw | vector length | neighbour distance | timestamp |
| ---- | -- | ------------ | ------------- | ------------------ | --------- |

Yaw is positive clockwise around the shared gravity axis and normalized to
$[-\pi, \pi)$. It is the relative orientation offset between the two drone
frames, not the world bearing of the displacement vector. Keeping the measured
length separate ensures loop closure can change yaw without changing range.
Only measured observations are transmitted. Each drone stores and applies its
own corrected yaw locally; corrected values are never accepted from peers.

```tikz
\usetikzlibrary{arrows.meta,calc}
\begin{document}
% ---- Catppuccin Latte ----
\definecolor{ctpBase} {HTML}{EFF1F5}
\definecolor{ctpMantle} {HTML}{E6E9EF}
\definecolor{ctpSurface1}{HTML}{BCC0CC}
\definecolor{ctpOverlay1}{HTML}{8C8FA1}
\definecolor{ctpOverlay0}{HTML}{9CA0B0}
\definecolor{ctpText} {HTML}{4C4F69}
\definecolor{ctpRed} {HTML}{D20F39}
\definecolor{ctpBlue} {HTML}{1E66F5}
\definecolor{ctpFlamingo}{HTML}{DD7878}
\definecolor{ctpLavender}{HTML}{7287FD}
\definecolor{ctpMaroon} {HTML}{8E0B2A}

\begin{tikzpicture}[
  node/.style={circle, draw=ctpBlue, fill=ctpBase, thick, minimum size=8mm, font=\small, text=ctpText},
  bridgenode/.style={circle, draw=ctpMaroon, fill=ctpBase, thick, minimum size=8mm, font=\small, text=ctpText},
  chainedge/.style={draw=ctpOverlay1, thick},
  meshedge/.style={draw=ctpLavender, thick},
  bridgeedge/.style={draw=ctpRed, thick}
]
  % chain side (line topology, rotated -45 degrees, shifted up near B), numbered 1-4
  \foreach \i in {1,2,3} {
    \node[node] (C\i) at ($(2,2.5) + (-45:\i*1.5)$) {\i};
  }
  \foreach \i [remember=\i as \last (initially 1)] in {2,3} {
    \draw[chainedge] (C\last) -- (C\i);
  }

  % bridge node B, numbered 4
  \node[bridgenode] (B) at (6.75, -1.5) {4};

  % mesh side (fully connected), numbered 5-8
  \foreach \i/\ang/\num in {1/90/5, 2/162/6, 3/234/7, 4/306/8} {
    \node[node] (M\i) at ($(9,0) + (\ang:1.6)$) {\num};
  }
  \foreach \i in {1,...,4} {
    \foreach \j in {1,...,4} {
      \ifnum\i<\j
        \draw[meshedge] (M\i) -- (M\j);
      \fi
    }
  }

  \draw[bridgeedge] (C3) -- (B);
  \draw[bridgeedge] (B) -- (M2);
  \draw[bridgeedge] (B) -- (M3);

\end{tikzpicture}
\end{document}
```

## Error accumulation & correction

By having each drone having a lookup table and calculating the position based on its own reading, we need to take into account that all our drones have a different heading creating a *yaw drift* for our system. We do not get this error for the pitch as our base vector (gravity) remains the same between all drones.

We can actually easily solve this by using our base as the absolute reference, meaning all our yaw references are calculated by triangulating our angle to the base and this new drone point. This is what we have taken into account and is the reason why all the vectors are based in our base reference frame.

If we would have multiple bases we could express the vector as a list of a key value pair.

One issue we get still have is that we have really just moved the *yaw drift* error, we actually haven't solved it since our drone relation to the base is still established through neighbours (unless a direct link is established) which is what pins the entire network. So we need to pair this solution with a *loop closure*. If we have a loop of drones we know that the yaw angles should sum to zero. Using this we would be able to remove residual errors that might persist after closest neighbour and absolute base vector correction.

The `connections` field describes every loop, while the directed measured-yaw
metadata above supplies the independent orientation observation for each edge.
Corrected yaw is local state and is not transmitted. We discover a loop by
walking connections until returning to an already visited drone ID.

For a loop of drones $d_1 \to d_2 \to \dots \to d_k \to d_1$ we take the relative yaw offset $\theta_{i,i+1}$ that each edge contributes and compose them around the loop:

$$
\theta_{err} = \sum_{i=1}^{k} \theta_{i,i+1}
$$

Because the only axis that can drift is yaw, this residual collapses to a single angle. In a perfect system $\theta_{err} = 0$, whatever we actually measure is the accumulated drift around that loop.

The sum is normalized to $[-\pi, \pi)$ before correction so equivalent full
turns do not create a false residual. A loop requires at least three distinct
drones; an immediate two-node backtrack is not a closure constraint.

We then distribute $\theta_{err}$ back across the edges of the loop to force it back to zero. We do **not** split it evenly, we weight the correction by the *neighbour distance* of each edge:
$$
\Delta\theta_{i,i+1} = \theta_{err} \times \frac{w_{i,i+1}}{\sum_j w_j}
\qquad
w_{i,i+1} = (d_{i,i+1} + 1)
$$
This means a long, high-neighbour-distance edge absorbs more of the correction, and a direct connection barely moves. This way, the *neighbour distance* is not replaced by the loop closure, it becomes the confidence weight that the loop closure uses to decide where the error most likely came from.

Note that the length of the vector is not covered by this and that is purely handled by the neighbour distance discrimination

## Coverage area

We will also send information about the coverage area, this is selected by the user in the frontend, the issue with the coverage area is that it can only exist in respect to the base as it will be the only location with static coordinates, but similarly if you have a vector to the base you can easily derive the coverage area.

For our hackathon we make this area static, given by the frontend user, but in the communication we will assume this area could change during the mission, hence why we send it with every broadcast.

We will structure it as follows:

| point | vector  |
| ----- | ------- |
| X     | vectorX |
| Y     | vectorY |
| Z     | vectorZ |

The area within the vectors should be easily derived by the drone, our base verifies that we need a minimum of 3 points to make an area for the front-end.

## Update rate

Since this math we use to translate vectors are trivial for our drones we can easily make the update rate quite extensive, but we need to make it fast enough for our worst case scenarios to work. Consider the following horseshoe mesh:

```tikz
\usetikzlibrary{arrows.meta,calc}
\begin{document}
% ---- Catppuccin Latte ----
\definecolor{ctpBase} {HTML}{EFF1F5}
\definecolor{ctpMantle} {HTML}{E6E9EF}
\definecolor{ctpSurface1}{HTML}{BCC0CC}
\definecolor{ctpOverlay1}{HTML}{8C8FA1}
\definecolor{ctpOverlay0}{HTML}{9CA0B0}
\definecolor{ctpText} {HTML}{4C4F69}
\definecolor{ctpRed} {HTML}{D20F39}
\definecolor{ctpBlue} {HTML}{1E66F5}
\definecolor{ctpFlamingo}{HTML}{DD7878}
\definecolor{ctpLavender}{HTML}{7287FD}
\definecolor{ctpMaroon} {HTML}{8E0B2A}

\begin{tikzpicture}[
  node/.style={circle, draw=ctpBlue, fill=ctpBase, thick, minimum size=6mm, font=\scriptsize, text=ctpText},
  ringedge/.style={draw=ctpOverlay1, thick}
]
  % ring of 20 nodes, numbered 1-20, node 1 and node 20 straddle the top
  \foreach \i in {1,...,20} {
    \pgfmathsetmacro{\ang}{99 - (\i-1)*18}
    \node[node] (R\i) at ($(0,0) + (\ang:3.5)$) {\i};
  }

  % ring connections between consecutive nodes, skipping the top pair (1--20)
  \foreach \i in {1,...,19} {
    \draw[ringedge] (R\i) -- (R\the\numexpr\i+1\relax);
  }
\end{tikzpicture}
\end{document}
```

In this situation node 20 has a delay of information of $19\times u_t$ seconds in relation to 1, where $u_t$ is our update rate. We can very easily graph this for our project:

```tikz
\usepackage{pgfplots}
\pgfplotsset{compat=1.16}

% ---- Catppuccin Latte palette ----
\definecolor{ctpMauve}{HTML}{8839ef}
\definecolor{ctpRed}{HTML}{d20f39}
\definecolor{ctpPeach}{HTML}{fe640b}
\definecolor{ctpGreen}{HTML}{40a02b}
\definecolor{ctpBlue}{HTML}{1e66f5}
\definecolor{ctpText}{HTML}{4c4f69}
\definecolor{ctpSubtext0}{HTML}{6c6f85}
\definecolor{ctpOverlay1}{HTML}{8c8fa1}
\definecolor{ctpBase}{HTML}{eff1f5}

\begin{document}
\begin{tikzpicture}
  \begin{axis}
    [
    title = {Worst-case propagation delay vs swarm size},
    title style={text=ctpText},
    axis lines = left,
    axis line style={ctpText},
    tick style={ctpSubtext0},
    xlabel style={text=ctpText},
    ylabel style={text=ctpText},
    xticklabel style={text=ctpText},
    yticklabel style={text=ctpText},
    xmin=0, xmax=100,
    domain=1:100,
    ymin=0, ymax=10,
    restrict y to domain=0:10,
    samples=200,
    no markers,
    xlabel = {neighbour distance $n$}, ylabel = {worst-case delay $D$ (s)},
    width=10cm,
    height=7cm,
    axis background/.style={fill=ctpBase},
    legend style={
      font=\footnotesize,
      at={(0.5,-0.2)}, anchor=north,
      legend columns=3,
      draw=none,
      fill=none,
      text=ctpText,
      column sep=1ex,
      },
    ]
    \addplot[ctpRed, thick]   {0.1  * (x+1)}; \addlegendentry{$u_t = 0.1$s}
    \addplot[ctpBlue, thick]  {0.05 * (x+1)}; \addlegendentry{$u_t = 0.05$s}
    \addplot[ctpGreen, thick] {0.01 * (x+1)}; \addlegendentry{$u_t = 0.01$s}
  \end{axis}
\end{tikzpicture}
\end{document}
```

Since a quad drone only has a max speed of $160km/h \approx 44m/s$ and our simulation won't use a major swarm, we concluded that 0.1 second is fine, since it would mean we have an error of 4.4 meters which is easily found using a [spiral search](seeking.md). For a larger cluster we would definitely consider 0.01, but at that point we would have to go over how our cpu will handle the parsing and the trigonometry.
