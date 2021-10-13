# GCOPTER

__GCOPTER__ is an efficient and versatile multicopter trajectory optimizer built upon a novel sparse trajectory representation named [__MINCO__](https://arxiv.org/pdf/2103.00190v2.pdf).

- To be open-sourced.

## About

__Author__: [Zhepei Wang](https://zhepeiwang.github.io/) and [Fei Gao](https://ustfei.com/) from [ZJU FAST Lab](http://zju-fast.com/).

__Related Paper__:

[Geometrically Constrained Trajectory Optimization for Multicopters](https://arxiv.org/abs/2103.00190v2), Zhepei Wang, Xin Zhou, Chao Xu, and Fei Gao, Under Review.
```
@article{WANG2021GCOPTER,
    title={Geometrically Constrained Trajectory Optimization for Multicopters},
    author={Wang, Zhepei and Zhou, Xin and Xu, Chao and Gao, Fei},
    journal={arXiv preprint arXiv:2103.00190},
    year={2021}
}
```

## Applications

- Robust Real-Time SE(3) Planning: [youtube](https://www.youtube.com/watch?v=pQ4oSf1rdBU) or [bilibili](https://www.bilibili.com/video/BV1bb4y1X7VE/). (__Reported by [IEEE Spectrum Website](https://spectrum.ieee.org/)!__)
<a href="https://www.youtube.com/watch?v=pQ4oSf1rdBU" target="blank">
    <p align="center">
        <img src="misc/gcopter_se3task_cover.png" width="600" height="337" />
    </p>
</a>

- Multicopter Swarms Planning: [youtube](https://www.youtube.com/watch?v=w5GDMpjAoVQ) or [bilibili](https://www.bilibili.com/video/BV1gK4y1g7F7). (__Also Reported by [IEEE Spectrum Website](https://spectrum.ieee.org/)!__)
<a href="https://www.youtube.com/watch?v=w5GDMpjAoVQ" target="blank">
    <p align="center">
        <img src="misc/gcopter_swarmtask_cover.png" width="600" height="337" />
    </p>
</a>

- Long-Distance Drone Racing Planning: [youtube](https://www.youtube.com/watch?v=oIqtN3zWIhM) or [bilibili](https://www.bilibili.com/video/BV1sq4y1779e). (__Published in [IEEE RAL](https://ieeexplore.ieee.org/document/9543598)__)
<a href="https://www.youtube.com/watch?v=oIqtN3zWIhM" target="blank">
    <p align="center">
        <img src="misc/gcopter_racingtask_cover.png" width="600" height="337" />
    </p>
</a>

- Gaze Teleoperation Planning: [youtube](https://www.youtube.com/watch?v=WYujLePQwB8) or [bilibili](https://www.bilibili.com/video/BV1Yf4y1P74v). (Submitted to IEEE RAL)
<a href="https://www.youtube.com/watch?v=WYujLePQwB8" target="blank">
    <p align="center">
        <img src="misc/minco_gazetele_cover.png" width="600" height="337" />
    </p>
</a>

- More applications powered by __GCOPTER__ or __MINCO__ will be listed here.

## Powerful Submodules
- [SDLP: Seidel's Algorithm](https://github.com/ZJU-FAST-Lab/SDLP) on Linear-Complexity Linear Programming for Computational Geometry.
- [VertexEnumeration3D](https://github.com/ZJU-FAST-Lab/VertexEnumeration3D): Highly Efficient Vertex Enumeration for 3D Convex Polytopes (Outperforms [cddlib](https://github.com/cddlib/cddlib) in 3D).
- [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite): An Easy-to-Use Header-Only L-BFGS Solver.
