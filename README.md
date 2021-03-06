# GCOPTER

__GCOPTER__ is a versatile multicopter trajectory optimizer built upon a novel sparse trajectory representation __MINCO__.

- To be open-sourced.

## 0. About

__Author__: [Zhepei Wang](https://zhepeiwang.github.io/) and [Fei Gao](https://ustfei.com/) from the [ZJU FAST Lab](http://zju-fast.com/).

__Related Paper__:

[Geometrically Constrained Trajectory Optimization for Multicopters](https://arxiv.org/abs/2103.00190), Zhepei Wang, Xin Zhou, Chao Xu, and Fei Gao, Under Review.
```
@article{WANG2021GCOPTER,
    title={Geometrically Constrained Trajectory Optimization for Multicopters},
    author={Wang, Zhepei and Zhou, Xin and Xu, Chao and Gao, Fei},
    journal={arXiv preprint arXiv:2103.00190},
    year={2021}
}
```

__Video__:

- Robust real-time SE(3) tasks: [youtube](https://youtu.be/pQ4oSf1rdBU) or [bilibili](https://www.bilibili.com/video/BV1bb4y1X7VE/).
<a href="https://youtu.be/pQ4oSf1rdBU" target="blank">
    <p align="center">
        <img src="misc/gcopter_se3task_cover.png" width="600" height="337" />
    </p>
</a>

- More tasks powered by __GCOPTER__ will be released here, including __dense dynamic obstacle__, __multicopter swarms__, and __large-scale drone racing__.

__Available Powerful Submodules__:
- [SDLP](https://github.com/ZJU-FAST-Lab/SDLP): [Seidel's Algorithm](https://link.springer.com/article/10.1007/BF02574699) on Linear-Complexity Linear Programming for Computational Geometry.
- [VertexEnumeration3D](https://github.com/ZJU-FAST-Lab/VertexEnumeration3D): Highly Efficient Vertex Enumeration for 3D Convex Polytopes (Outperforms [cddlib](https://github.com/cddlib/cddlib) in 3D).
- [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite): An Easy-to-Use Header-Only L-BFGS Solver.
