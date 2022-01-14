
<div align="center">
  <a href="http://erl.ucsd.edu/">
    <img align="left" src="docs/media/erl.png" width="80" alt="erl">
  </a>
  <a href="https://contextualrobotics.ucsd.edu/">
    <img align="center" src="docs/media/cri.png" width="150" alt="cri">
  </a>
  <a href="https://ucsd.edu/">
    <img align="right" src="docs/media/ucsd.png" width="260" alt="ucsd">
  </a>
</div>

# Semantic Shannon Mutual Information (S2MI)

S2MI is a library for autonomous robot exploration using a stream of depth and semantic segmentation images as the input to build a semantically annotated OctoMap in real-time.

S2MI is implemented as two ROS packages, which can be built on x86-64 and ARM-based processors:
- Semantic OctoMap implementation for building probabilistic multi-class maps of an environment ([S2MI-Mapping](https://github.com/ExistentialRobotics/S2MI/tree/master/S2MI-Mapping))
- Autonomous exploration using Semantic Shannon Mutual Information ([S2MI-Planning](https://github.com/ExistentialRobotics/S2MI/tree/master/S2MI-Planning))

<p align="center">
    <a href="https://www.youtube.com/embed/KosSp3znqFo">
    <img src="docs/media/s2mi_thumbnail.png" alt="S2MI">
    </a>
</p>


## Citation

If you found this work useful, we would appreciate if you could cite our work:

- [1] A. Asgharivaskasi, N. Atanasov. [**Active Bayesian Multi-class Mapping from Range and Semantic Segmentation Observations**](https://ieeexplore.ieee.org/document/9561711). IEEE Int. Conf. on Robotics and Automation (ICRA), 2021.

```bibtex
@InProceedings{Asgharivaskasi-ICRA21,
  author={Asgharivaskasi, Arash and Atanasov, Nikolay},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Active Bayesian Multi-class Mapping from Range and Semantic Segmentation Observations}, 
  year={2021},
  volume={},
  number={},
  pages={1-7}
```

- [2] A. Asgharivaskasi, N. Atanasov. [**Active Bayesian Multi-class Mapping from Range and Semantic Segmentation Observations**](https://arxiv.org/abs/2112.04063). [arXiv:2112.04063
](https://arxiv.org/abs/2112.04063).
 
 ```bibtex
@InProceedings{Asgharivaskasi-journal,
  title = {Active Bayesian Multi-class Mapping from Range and Semantic Segmentation Observations},
  author = {A. Asgharivaskasi, N. Atanasov},
  year = {2021},
  booktitle = {arxiv},
  pdf = {https://arxiv.org/pdf/2112.04063.pdf}
}
```

## Acknowledgments

We gratefully acknowledge support from ARL [DCIST](https://www.dcist.org/) CRA W911NF-17-2-0181 and [NSF](https://www.nsf.gov/) FRR CAREER 2045945.

## License

[BSD License](LICENSE.BSD)
