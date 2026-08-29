# Seeking

Spiral search is an algorithm used to search from a point outward in expanding spirals. The algorithm is quite simple, below we have a visualisation in 3d.

```tikz
\usetikzlibrary{arrows.meta,calc}
\begin{document}
% ---- Catppuccin Latte ----
\definecolor{ctpBase}    {HTML}{EFF1F5}
\definecolor{ctpMantle}  {HTML}{E6E9EF}
\definecolor{ctpBase} {HTML}{EFF1F5}
\definecolor{ctpMantle} {HTML}{E6E9EF}
\definecolor{ctpSurface1}{HTML}{BCC0CC}
\definecolor{ctpOverlay1}{HTML}{8C8FA1}
\definecolor{ctpOverlay0}{HTML}{9CA0B0}
\definecolor{ctpText}    {HTML}{4C4F69}
\definecolor{ctpRed}     {HTML}{D20F39}
\definecolor{ctpBlue}    {HTML}{1E66F5}
\definecolor{ctpText} {HTML}{4C4F69}
\definecolor{ctpRed} {HTML}{D20F39}
\definecolor{ctpBlue} {HTML}{1E66F5}
\definecolor{ctpFlamingo}{HTML}{DD7878}
\definecolor{ctpLavender}{HTML}{7287FD}
\definecolor{ctpMaroon} {HTML}{8E0B2A}
% ---- reusable cone macro: #1#2#3=apex P, #4#5#6=base center Q, #7=base radius, #8=color ----
\newcommand{\drawCone}[8]{%
  \pgfmathsetmacro{\Px}{#1}\pgfmathsetmacro{\Py}{#2}\pgfmathsetmacro{\Pz}{#3}%
  \pgfmathsetmacro{\Qx}{#4}\pgfmathsetmacro{\Qy}{#5}\pgfmathsetmacro{\Qz}{#6}%
  \pgfmathsetmacro{\rad}{#7}%
  \pgfmathsetmacro{\ddx}{\Qx-\Px}\pgfmathsetmacro{\ddy}{\Qy-\Py}\pgfmathsetmacro{\ddz}{\Qz-\Pz}%
  \pgfmathsetmacro{\dlen}{sqrt(\ddx*\ddx+\ddy*\ddy+\ddz*\ddz)}%
  % u = d x (0,0,1)
  \pgfmathsetmacro{\ux}{\ddy}\pgfmathsetmacro{\uy}{-\ddx}\pgfmathsetmacro{\uz}{0}%
  \pgfmathsetmacro{\ulen}{sqrt(\ux*\ux+\uy*\uy+\uz*\uz)}%
  \pgfmathsetmacro{\uhx}{\ux/\ulen}\pgfmathsetmacro{\uhy}{\uy/\ulen}\pgfmathsetmacro{\uhz}{\uz/\ulen}%
  \pgfmathsetmacro{\dhx}{\ddx/\dlen}\pgfmathsetmacro{\dhy}{\ddy/\dlen}\pgfmathsetmacro{\dhz}{\ddz/\dlen}%
  % v = dhat x uhat (orthonormal to both)
  \pgfmathsetmacro{\vhx}{\dhy*\uhz-\dhz*\uhy}%
  \pgfmathsetmacro{\vhy}{\dhz*\uhx-\dhx*\uhz}%
  \pgfmathsetmacro{\vhz}{\dhx*\uhy-\dhy*\uhx}%
  % translucent mantle: fan of triangles from P to the base circle
  \foreach \t in {0,15,...,345} {
    \pgfmathsetmacro{\tb}{\t+15}
    \pgfmathsetmacro{\pax}{\Qx+\rad*(cos(\t)*\uhx+sin(\t)*\vhx)}
    \pgfmathsetmacro{\pay}{\Qy+\rad*(cos(\t)*\uhy+sin(\t)*\vhy)}
    \pgfmathsetmacro{\paz}{\Qz+\rad*(cos(\t)*\uhz+sin(\t)*\vhz)}
    \pgfmathsetmacro{\pbx}{\Qx+\rad*(cos(\tb)*\uhx+sin(\tb)*\vhx)}
    \pgfmathsetmacro{\pby}{\Qy+\rad*(cos(\tb)*\uhy+sin(\tb)*\vhy)}
    \pgfmathsetmacro{\pbz}{\Qz+\rad*(cos(\tb)*\uhz+sin(\tb)*\vhz)}
    \fill[#8, opacity=0.12] (#1,#2,#3) -- (\pax,\pay,\paz) -- (\pbx,\pby,\pbz) -- cycle;
    \draw[#8, opacity=0.35, thin] (#1,#2,#3) -- (\pax,\pay,\paz);
  }
  % translucent base disk
  \fill[#8, opacity=0.18] plot[variable=\t,domain=0:360,samples=48]
    ({\Qx+\rad*(cos(\t)*\uhx+sin(\t)*\vhx)},
     {\Qy+\rad*(cos(\t)*\uhy+sin(\t)*\vhy)},
     {\Qz+\rad*(cos(\t)*\uhz+sin(\t)*\vhz)}) -- cycle;
  % rim outline, split into near/far halves so it reads as 3D:
  % assumed view direction (looking from the +x+y+z octant toward the scene)
  \pgfmathsetmacro{\Wx}{1}\pgfmathsetmacro{\Wy}{1}\pgfmathsetmacro{\Wz}{1}%
  \pgfmathsetmacro{\Ac}{\uhx*\Wx+\uhy*\Wy+\uhz*\Wz}%
  \pgfmathsetmacro{\Bc}{\vhx*\Wx+\vhy*\Wy+\vhz*\Wz}%
  \pgfmathsetmacro{\phi}{atan2(\Bc,\Ac)}%
  % far half of the rim: faces away from viewer, tucked behind the cone body -> darker
  \draw[#8!55!black, thick, opacity=0.7] plot[variable=\t,domain={\phi+90}:{\phi+270},samples=48]
    ({\Qx+\rad*(cos(\t)*\uhx+sin(\t)*\vhx)},
     {\Qy+\rad*(cos(\t)*\uhy+sin(\t)*\vhy)},
     {\Qz+\rad*(cos(\t)*\uhz+sin(\t)*\vhz)});
  % near half of the rim: faces the viewer, sits over/in front of the cone body -> normal, crisp
  \draw[#8, thick, opacity=0.9] plot[variable=\t,domain={\phi-90}:{\phi+90},samples=48]
    ({\Qx+\rad*(cos(\t)*\uhx+sin(\t)*\vhx)},
     {\Qy+\rad*(cos(\t)*\uhy+sin(\t)*\vhy)},
     {\Qz+\rad*(cos(\t)*\uhz+sin(\t)*\vhz)});
}
% ---- reusable ring macro (like 2d inset circle): #1#2#3=P, #4#5#6=Q, #7=ring radius, #8=color, #9=start angle deg ----
\newcommand{\drawRing}[9]{%
  \pgfmathsetmacro{\Px}{#1}\pgfmathsetmacro{\Py}{#2}\pgfmathsetmacro{\Pz}{#3}%
  \pgfmathsetmacro{\Qx}{#4}\pgfmathsetmacro{\Qy}{#5}\pgfmathsetmacro{\Qz}{#6}%
  \pgfmathsetmacro{\radR}{#7}%
  \pgfmathsetmacro{\ddx}{\Qx-\Px}\pgfmathsetmacro{\ddy}{\Qy-\Py}\pgfmathsetmacro{\ddz}{\Qz-\Pz}%
  \pgfmathsetmacro{\ux}{\ddy}\pgfmathsetmacro{\uy}{-\ddx}\pgfmathsetmacro{\uz}{0}%
  \pgfmathsetmacro{\ulen}{sqrt(\ux*\ux+\uy*\uy+\uz*\uz)}%
  \pgfmathsetmacro{\uhx}{\ux/\ulen}\pgfmathsetmacro{\uhy}{\uy/\ulen}\pgfmathsetmacro{\uhz}{\uz/\ulen}%
  \pgfmathsetmacro{\dlen}{sqrt(\ddx*\ddx+\ddy*\ddy+\ddz*\ddz)}%
  \pgfmathsetmacro{\dhx}{\ddx/\dlen}\pgfmathsetmacro{\dhy}{\ddy/\dlen}\pgfmathsetmacro{\dhz}{\ddz/\dlen}%
  \pgfmathsetmacro{\vhx}{\dhy*\uhz-\dhz*\uhy}%
  \pgfmathsetmacro{\vhy}{\dhz*\uhx-\dhx*\uhz}%
  \pgfmathsetmacro{\vhz}{\dhx*\uhy-\dhy*\uhx}%
  \draw[#8, -{Stealth}, very thick] plot[variable=\t,domain=#9:{#9+180},samples=48]
    ({\Qx+\radR*(cos(\t)*\uhx+sin(\t)*\vhx)},
     {\Qy+\radR*(cos(\t)*\uhy+sin(\t)*\vhy)},
     {\Qz+\radR*(cos(\t)*\uhz+sin(\t)*\vhz)});
}
% ---- reusable spiral-arrow macro (3D version of the 2d inset spiral): #1#2#3=P, #4#5#6=Q, #7=max radius, #8=color, #9=degrees swept ----
\newcommand{\drawSpiralArrow}[9]{%
  \pgfmathsetmacro{\Px}{#1}\pgfmathsetmacro{\Py}{#2}\pgfmathsetmacro{\Pz}{#3}%
  \pgfmathsetmacro{\Qx}{#4}\pgfmathsetmacro{\Qy}{#5}\pgfmathsetmacro{\Qz}{#6}%
  \pgfmathsetmacro{\radMax}{#7}%
  \pgfmathsetmacro{\ddx}{\Qx-\Px}\pgfmathsetmacro{\ddy}{\Qy-\Py}\pgfmathsetmacro{\ddz}{\Qz-\Pz}%
  \pgfmathsetmacro{\ux}{\ddy}\pgfmathsetmacro{\uy}{-\ddx}\pgfmathsetmacro{\uz}{0}%
  \pgfmathsetmacro{\ulen}{sqrt(\ux*\ux+\uy*\uy+\uz*\uz)}%
  \pgfmathsetmacro{\uhx}{\ux/\ulen}\pgfmathsetmacro{\uhy}{\uy/\ulen}\pgfmathsetmacro{\uhz}{\uz/\ulen}%
  \pgfmathsetmacro{\dlen}{sqrt(\ddx*\ddx+\ddy*\ddy+\ddz*\ddz)}%
  \pgfmathsetmacro{\dhx}{\ddx/\dlen}\pgfmathsetmacro{\dhy}{\ddy/\dlen}\pgfmathsetmacro{\dhz}{\ddz/\dlen}%
  \pgfmathsetmacro{\vhx}{\dhy*\uhz-\dhz*\uhy}%
  \pgfmathsetmacro{\vhy}{\dhz*\uhx-\dhx*\uhz}%
  \pgfmathsetmacro{\vhz}{\dhx*\uhy-\dhy*\uhx}%
  \draw[#8, -{Stealth}, thick] plot[variable=\t,domain=0:{#9-180},samples=200,smooth]
    ({\Qx+(\radMax*\t/#9)*(cos(\t)*\uhx+sin(\t)*\vhx)},
     {\Qy+(\radMax*\t/#9)*(cos(\t)*\uhy+sin(\t)*\vhy)},
     {\Qz+(\radMax*\t/#9)*(cos(\t)*\uhz+sin(\t)*\vhz)});
}
\begin{tikzpicture}[
x={(0.4cm,0.2cm)}, y={(1cm,0cm)}, z={(0cm,1cm)},
text=ctpText,
vec/.style={-{Stealth[length=3mm]}, very thick, shorten >=2pt},
drop/.style={dashed, ctpOverlay0, thin},
foot/.style={circle, fill=ctpOverlay1, inner sep=1.1pt},
dot/.style={circle, fill=ctpText, inner sep=1.5pt}
]
% === original scene ===
\fill[ctpMantle] (0,0,0) -- (7,0,0) -- (7,6,0) -- (0,6,0) -- cycle;
\foreach \i in {0,...,7} \draw[ctpSurface1, very thin] (\i,0,0) -- (\i,6,0);
\foreach \j in {0,...,6} \draw[ctpSurface1, very thin] (0,\j,0) -- (7,\j,0);
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (7.8,0,0) node[below right] {x};
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (0,6.8,0) node[right] {y};
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (0,0,4.4) node[above] {z};
\coordinate (P) at (1,1,2);
\coordinate (Q) at (6,2,3);
% ---- 3D translucent cones: apex P, base Q. Call \drawCone{Px}{Py}{Pz}{Qx}{Qy}{Qz}{rad}{color} ----
\begin{scope}
\drawCone{1}{1}{2}{6}{2}{3}{1}{ctpRed}    % outer cone, radius 1
\drawCone{1}{1}{2}{6}{2}{3}{0.125}{ctpMaroon} % inner nested cone, radius 0.125 (50% of prior 0.25), darker red
\drawSpiralArrow{1}{1}{2}{6}{2}{3}{1}{black}{1080} % spiral scan arrow, 3 turns out to outer cone radius (more compact/rotation than before)
\end{scope}
\node[dot] at (P) {}; \node[left] at (P) {P};
\node[dot] at (Q) {}; \node[right] at (Q) {Q};
\end{tikzpicture}
\end{document}
```

The graph has two cones sharing apex the outer (red) is our full search angle, while the inner (maroon) is where our beam is currently pointing. The arrow shows only the inner beam's rotation as it spirals outward to cover the search angle.

For our purposes spiral search is the perfect search algorithm, by using our [lookup table](communication.md#lookup-table) we are given a vector which tells us where the other drone is located, as well as how current this data is (not accounting for our clock drifting). Since this data could be slightly out of date, we then can use the spiral search to slowly extend our search outwards. We can calculate the size of our outer cone with the following equation:
$$
\theta = 2 \times \arctan\left(\frac{\Delta r}{R} \right)
$$
where:
$$
\begin{eqnarray}
\theta :&& \text{search angle} \\
\Delta r :&& \text{position uncertainty radius} = v_{rel} \times \Delta t \\
v_{rel} :&& \text{worst-case relative speed} = 2 \times v_{max} \text{ (both drones separating head-on)} \\
R :&& \text{Length of vector from drone $P$ to $Q$}
\end{eqnarray}
$$
note that $v_{max}$ is the max velocity of our drones and $\Delta t$ is the timestamp difference.

This drone search width can easily be calculated on the fly. To give you an idea, below are some values:

| $\Delta t$ (time since last fix) | $\Delta r = v_{rel} \times \Delta t$ | $\theta = 2\arctan(\Delta r / R)$ |
| -------------------------------- | ------------------------------------ | --------------------------------- |
| $0.1$ s (1 tick)                 | $5.6$ m                              | $0.21°$                           |
| $0.5$ s (5 ticks)                | $27.8$ m                             | $1.1°$                            |
| $2$ s (20 ticks)                 | $111$ m                              | $4.2°$                            |
| $10$ s (100 ticks)               | $556$ m                              | $21°$                             |
| $50$ s (500 ticks, worst case)   | $2778$ m                             | $86°$                             |

Note that a tick is one update cycle. For our cases it's every 0.1 seconds as stated in [communication](communication.md).

An issue which we might experience in the search process is that while our cones overlap, if the spiral search has the same rotation speed on both drones we might never establish a connection. Illustrated below:

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
% ---- reusable cone macro: #1#2#3=apex P, #4#5#6=base center Q, #7=base radius, #8=color ----
\newcommand{\drawCone}[8]{%
  \pgfmathsetmacro{\Px}{#1}\pgfmathsetmacro{\Py}{#2}\pgfmathsetmacro{\Pz}{#3}%
  \pgfmathsetmacro{\Qx}{#4}\pgfmathsetmacro{\Qy}{#5}\pgfmathsetmacro{\Qz}{#6}%
  \pgfmathsetmacro{\rad}{#7}%
  \pgfmathsetmacro{\ddx}{\Qx-\Px}\pgfmathsetmacro{\ddy}{\Qy-\Py}\pgfmathsetmacro{\ddz}{\Qz-\Pz}%
  \pgfmathsetmacro{\dlen}{sqrt(\ddx*\ddx+\ddy*\ddy+\ddz*\ddz)}%
  % u = d x (0,0,1)
  \pgfmathsetmacro{\ux}{\ddy}\pgfmathsetmacro{\uy}{-\ddx}\pgfmathsetmacro{\uz}{0}%
  \pgfmathsetmacro{\ulen}{sqrt(\ux*\ux+\uy*\uy+\uz*\uz)}%
  \pgfmathsetmacro{\uhx}{\ux/\ulen}\pgfmathsetmacro{\uhy}{\uy/\ulen}\pgfmathsetmacro{\uhz}{\uz/\ulen}%
  \pgfmathsetmacro{\dhx}{\ddx/\dlen}\pgfmathsetmacro{\dhy}{\ddy/\dlen}\pgfmathsetmacro{\dhz}{\ddz/\dlen}%
  % v = dhat x uhat (orthonormal to both)
  \pgfmathsetmacro{\vhx}{\dhy*\uhz-\dhz*\uhy}%
  \pgfmathsetmacro{\vhy}{\dhz*\uhx-\dhx*\uhz}%
  \pgfmathsetmacro{\vhz}{\dhx*\uhy-\dhy*\uhx}%
  % translucent mantle: fan of triangles from P to the base circle
  \foreach \t in {0,15,...,345} {
    \pgfmathsetmacro{\tb}{\t+15}
    \pgfmathsetmacro{\pax}{\Qx+\rad*(cos(\t)*\uhx+sin(\t)*\vhx)}
    \pgfmathsetmacro{\pay}{\Qy+\rad*(cos(\t)*\uhy+sin(\t)*\vhy)}
    \pgfmathsetmacro{\paz}{\Qz+\rad*(cos(\t)*\uhz+sin(\t)*\vhz)}
    \pgfmathsetmacro{\pbx}{\Qx+\rad*(cos(\tb)*\uhx+sin(\tb)*\vhx)}
    \pgfmathsetmacro{\pby}{\Qy+\rad*(cos(\tb)*\uhy+sin(\tb)*\vhy)}
    \pgfmathsetmacro{\pbz}{\Qz+\rad*(cos(\tb)*\uhz+sin(\tb)*\vhz)}
    \fill[#8, opacity=0.12] (#1,#2,#3) -- (\pax,\pay,\paz) -- (\pbx,\pby,\pbz) -- cycle;
    \draw[#8, opacity=0.35, thin] (#1,#2,#3) -- (\pax,\pay,\paz);
  }
  % translucent base disk
  \fill[#8, opacity=0.18] plot[variable=\t,domain=0:360,samples=48]
    ({\Qx+\rad*(cos(\t)*\uhx+sin(\t)*\vhx)},
     {\Qy+\rad*(cos(\t)*\uhy+sin(\t)*\vhy)},
     {\Qz+\rad*(cos(\t)*\uhz+sin(\t)*\vhz)}) -- cycle;
  % base circle outline
  \draw[#8, thick, opacity=0.85] plot[variable=\t,domain=0:360,samples=48]
    ({\Qx+\rad*(cos(\t)*\uhx+sin(\t)*\vhx)},
     {\Qy+\rad*(cos(\t)*\uhy+sin(\t)*\vhy)},
     {\Qz+\rad*(cos(\t)*\uhz+sin(\t)*\vhz)});
}
% ---- reusable ring macro (like 2d inset circle): #1#2#3=P, #4#5#6=Q, #7=ring radius, #8=color, #9=start angle deg ----
\newcommand{\drawRing}[9]{%
  \pgfmathsetmacro{\Px}{#1}\pgfmathsetmacro{\Py}{#2}\pgfmathsetmacro{\Pz}{#3}%
  \pgfmathsetmacro{\Qx}{#4}\pgfmathsetmacro{\Qy}{#5}\pgfmathsetmacro{\Qz}{#6}%
  \pgfmathsetmacro{\radR}{#7}%
  \pgfmathsetmacro{\ddx}{\Qx-\Px}\pgfmathsetmacro{\ddy}{\Qy-\Py}\pgfmathsetmacro{\ddz}{\Qz-\Pz}%
  \pgfmathsetmacro{\ux}{\ddy}\pgfmathsetmacro{\uy}{-\ddx}\pgfmathsetmacro{\uz}{0}%
  \pgfmathsetmacro{\ulen}{sqrt(\ux*\ux+\uy*\uy+\uz*\uz)}%
  \pgfmathsetmacro{\uhx}{\ux/\ulen}\pgfmathsetmacro{\uhy}{\uy/\ulen}\pgfmathsetmacro{\uhz}{\uz/\ulen}%
  \pgfmathsetmacro{\dlen}{sqrt(\ddx*\ddx+\ddy*\ddy+\ddz*\ddz)}%
  \pgfmathsetmacro{\dhx}{\ddx/\dlen}\pgfmathsetmacro{\dhy}{\ddy/\dlen}\pgfmathsetmacro{\dhz}{\ddz/\dlen}%
  \pgfmathsetmacro{\vhx}{\dhy*\uhz-\dhz*\uhy}%
  \pgfmathsetmacro{\vhy}{\dhz*\uhx-\dhx*\uhz}%
  \pgfmathsetmacro{\vhz}{\dhx*\uhy-\dhy*\uhx}%
  \draw[#8, -{Stealth}, very thick] plot[variable=\t,domain=#9:{#9+180},samples=48]
    ({\Qx+\radR*(cos(\t)*\uhx+sin(\t)*\vhx)},
     {\Qy+\radR*(cos(\t)*\uhy+sin(\t)*\vhy)},
     {\Qz+\radR*(cos(\t)*\uhz+sin(\t)*\vhz)});
}
% ---- reusable scene: same cones/grid/nodes, draw twice at different perspectives ----
\newcommand{\droneScene}{
\fill[ctpMantle] (0,0,0) -- (7,0,0) -- (7,6,0) -- (0,6,0) -- cycle;
\foreach \i in {0,...,7} \draw[ctpSurface1, very thin, opacity=0.4] (\i,0,0) -- (\i,6,0);
\foreach \j in {0,...,6} \draw[ctpSurface1, very thin, opacity=0.4] (0,\j,0) -- (7,\j,0);
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (7.8,0,0) node[below right] {x};
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (0,6.8,0) node[right] {y};
\draw[-{Stealth}, ctpOverlay1] (0,0,0) -- (0,0,4.4) node[above] {z};
\coordinate (P) at (1,1,2);
\coordinate (Q) at (6,2,3);
% ---- 3D translucent cones: apex stays at P/Q, base disk shifted left (-x) so both cones tilt the same way and overlap near the middle ----
\draw[ctpOverlay0, dashed, thin] (P) -- (Q); % reference line, makes the tilt/overlap of both cones off this axis easier to read
\begin{scope}
\drawCone{1}{1}{2}{5.75}{2}{3}{1}{ctpRed}    % outer cone, apex P, base tilted -0.25 left of Q (drone A's search cone)
\drawCone{6}{2}{3}{2}{1}{2}{1}{ctpBlue}  % reverse cone, apex Q, base tilted +1 right of P (drone B's search cone)
% red cone rim: near/far split outline (same effect as first graph), red only, not applied to blue
\pgfmathsetmacro{\rPx}{1}\pgfmathsetmacro{\rPy}{1}\pgfmathsetmacro{\rPz}{2}%
\pgfmathsetmacro{\rQx}{5.75}\pgfmathsetmacro{\rQy}{2}\pgfmathsetmacro{\rQz}{3}%
\pgfmathsetmacro{\rrad}{1}%
\pgfmathsetmacro{\rddx}{\rQx-\rPx}\pgfmathsetmacro{\rddy}{\rQy-\rPy}\pgfmathsetmacro{\rddz}{\rQz-\rPz}%
\pgfmathsetmacro{\rux}{\rddy}\pgfmathsetmacro{\ruy}{-\rddx}\pgfmathsetmacro{\ruz}{0}%
\pgfmathsetmacro{\rulen}{sqrt(\rux*\rux+\ruy*\ruy+\ruz*\ruz)}%
\pgfmathsetmacro{\ruhx}{\rux/\rulen}\pgfmathsetmacro{\ruhy}{\ruy/\rulen}\pgfmathsetmacro{\ruhz}{\ruz/\rulen}%
\pgfmathsetmacro{\rdlen}{sqrt(\rddx*\rddx+\rddy*\rddy+\rddz*\rddz)}%
\pgfmathsetmacro{\rdhx}{\rddx/\rdlen}\pgfmathsetmacro{\rdhy}{\rddy/\rdlen}\pgfmathsetmacro{\rdhz}{\rddz/\rdlen}%
\pgfmathsetmacro{\rvhx}{\rdhy*\ruhz-\rdhz*\ruhy}%
\pgfmathsetmacro{\rvhy}{\rdhz*\ruhx-\rdhx*\ruhz}%
\pgfmathsetmacro{\rvhz}{\rdhx*\ruhy-\rdhy*\ruhx}%
\pgfmathsetmacro{\rWx}{1}\pgfmathsetmacro{\rWy}{1}\pgfmathsetmacro{\rWz}{1}%
\pgfmathsetmacro{\rAc}{\ruhx*\rWx+\ruhy*\rWy+\ruhz*\rWz}%
\pgfmathsetmacro{\rBc}{\rvhx*\rWx+\rvhy*\rWy+\rvhz*\rWz}%
\pgfmathsetmacro{\rphi}{atan2(\rBc,\rAc)}%
\draw[ctpRed!55!black, thick, opacity=0.7] plot[variable=\t,domain={\rphi+90}:{\rphi+270},samples=48]
  ({\rQx+\rrad*(cos(\t)*\ruhx+sin(\t)*\rvhx)},
   {\rQy+\rrad*(cos(\t)*\ruhy+sin(\t)*\rvhy)},
   {\rQz+\rrad*(cos(\t)*\ruhz+sin(\t)*\rvhz)});
\draw[ctpRed, thick, opacity=0.9] plot[variable=\t,domain={\rphi-90}:{\rphi+90},samples=48]
  ({\rQx+\rrad*(cos(\t)*\ruhx+sin(\t)*\rvhx)},
   {\rQy+\rrad*(cos(\t)*\ruhy+sin(\t)*\rvhy)},
   {\rQz+\rrad*(cos(\t)*\ruhz+sin(\t)*\rvhz)});
\end{scope}
\node[dot] at (P) {}; \node[left] at (P) {P};
\node[dot] at (Q) {}; \node[right] at (Q) {Q};
}
\begin{tikzpicture}[
text=ctpText,
vec/.style={-{Stealth[length=3mm]}, very thick, shorten >=2pt},
drop/.style={dashed, ctpOverlay0, thin},
foot/.style={circle, fill=ctpOverlay1, inner sep=1.1pt},
dot/.style={circle, fill=ctpText, inner sep=1.5pt}
]
% === twisted perspective (x/y swapped vs a plain view) ===
\begin{scope}[x={(0.4cm,0.2cm)}, y={(1cm,0cm)}, z={(0cm,1cm)}]
\droneScene
\end{scope}
\end{tikzpicture}
\end{document}
```

Since none of these cones are centered around the point and are tilted in different amounts, we have a possibility that our searches aren't fully going to align, ie it will rotate in *lockstep* and they might miss each other entirely if they have the same search going at the same rate. To solve for this we can instead make the spiral search have a unique search speed based on the *ID* of our drones, allowing us to create the Weyl-hash formula, shown below:
$$
\omega_i = \omega_{min} + \operatorname{frac}(ID_i \cdot \varphi)\,(\omega_{max}-\omega_{min})
$$
where:
$$
\begin{eqnarray}
\omega_i :&& \text{angular scan speed assigned to drone } i \\
ID_i :&& \text{drone's unique integer ID} \\
\omega_{min}, \omega_{max} :&& \text{mechanical/electronic scan speed limits, e.g. } 0.5 \text{ rad/s (} \approx 4.8 \text{ rpm) to } 6 \text{ rad/s (} \approx 57 \text{ rpm)} \\
\varphi:&& \text{golden ratio, (the "most irrational" number) giving the most even spread with least clustering} \\
\operatorname{frac}(x)&& = x - \lfloor x \rfloor
\end{eqnarray}
$$

Note that our IDs are just a number for our hackathon, in a more professional situation you would have a unique UUID or similar, in that case we would probably only use the ASCII value of the name or something similar.
Also we used the golden ratio as the alternative would require us to know the total number of drones in our system which we could not guarantee, unless we constantly send that information to the network, but even then we might hit race conditions.

Once a lock has been established, we simply use our conical scan algorithm to maintain it.

## Why spiral search?

We selected spiral search as we believe it creates the least amount of searching required. Because of our [constant communication](communication.md) we can guarantee that each drone has a relatively up to date lookup table to be able to search for its peer, therefore making sure that we first check the vector and then spiralling outwards creates the least amount of searching needed.

Other alternatives we considered was a rolling shutter-like algorithm where we would start at the top right and go row by row to create a heat map of the area, this would also be useful if the antennas would have dual usage as radar antennas, giving our drones an ability to not only provide communication but also surveillance, this was deemed too advanced for the hackathon.
