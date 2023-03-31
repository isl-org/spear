![SPEAR](https://user-images.githubusercontent.com/2341965/229026429-38d957b4-9259-448d-8fb6-c1fe5b24aaab.jpg)

# SPEAR: A Simulator for Photorealistic Embodied AI Research

Interactive simulators are becoming powerful tools for training embodied agents, but existing simulators suffer from limited content diversity, physical interactivity, and visual fidelity. We address these limitations by introducing SPEAR: A Simulator for Photorealistic Embodied AI Research. To create our simulator, we worked closely with a team of professional artists for over a year to construct 300 unique virtual indoor environments with 2,566 unique rooms and 17,234 unique objects that can be manipulated individually. Each of our environments features detailed geometry, photorealistic materials, and a unique floor plan and object layout designed by a professional artist, i.e., we do not rely on remixing existing layouts to create additional content. Our environments are implemented as Unreal Engine assets, and we provide an OpenAI Gym interface for interacting with the environments via Python.

The SPEAR code is released under an [MIT License](LICENSE.txt), and the SPEAR assets are released under various [licenses](#licenses) that permit academic use.

## Citation

If you find SPEAR useful in your research, please cite this repository as follows:

```
@misc{roberts:2022,
    author       = {Mike Roberts AND Quentin Leboutet AND Rachith Prakash AND Renhan Wang AND
                    Hailin Zhang AND Rui Tang AND Marti Ferragut AND Stefan Leutenegger AND
                    Stephan R. Richter AND Vladlen Koltun AND Matthias M{\"u}ller AND German Ros},
    title        = {{SPEAR}: {A} Simulator for Photorealistic Embodied AI Research},
    howpublished = {\url{http://github.com/isl-org/spear}},
    year         = {2022},
}
```

## Getting Started

### Minimum and recommended system specifications

Minimum and recommended system specifications for the Unreal Engine are given [here](http://docs.unrealengine.com/4.26/en-US/Basics/RecommendedSpecifications).

### Precompiled binaries

See our [latest release notes](https://github.com/isl-org/spear/releases/tag/v0.2.0) for download links.

The easiest way to start working with SPEAR is to download a precompiled binary for your platform. Our precompiled binaries come pre-packaged with the scene pictured above. You can start interactively navigating around this scene with the keyboard and mouse simply by running the downloaded binary with no additional arguments.

### Working with multiple scenes

See our [Getting Started](docs/getting_started.md) tutorial.

### Programmatically interacting with SPEAR via Python 

See our [Getting Started](docs/getting_started.md) tutorial.

### Building from source

See our [Building SpearSim](docs/building_spearsim.md) tutorial.

Note that building from source is optional because we provide links to precompiled binaries.

### Contributing

See our [Contribution Guidelines](CONTRIBUTING.md) tutorial.

## Licenses

- The SPEAR code in this repository ([MIT License](LICENSE.txt))
- All of our external code dependencies (see [here](ACKNOWLEDGMENTS.txt))
- The OpenBot and Fetch assets included in this repository ([CC0 Creative Commons License](http://creativecommons.org/publicdomain/zero/1.0))
- The external assets referenced in `starter_content_0000` ([Unreal Engine End-User License Agreement](https://www.unrealengine.com/en-US/eula/unreal))
- The `starter_content_0000` scene included in this repository ([CC0 Creative Commons License](http://creativecommons.org/publicdomain/zero/1.0))
- The `warehouse_0000` scene ([CC0 Creative Commons License](http://creativecommons.org/publicdomain/zero/1.0))
- The `kujiale_0000` scene (see [here](LICENSE_KUJIALE.txt))
