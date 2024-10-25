# 3D Map Generator

This repository contains a Python script to generate 3D maps in Collada format from orthophoto mosaic images and 3D models of Switzerland provided by [Swisstopo](https://www.swisstopo.admin.ch/en/). The script will also produce two JPEG images: one showing a textured view of the 3D model and another with a semantic view.

## Data Sources

The data sources used to generate the 3D maps are:

- **Orthophotomosaic**: [Orthophotomosaic Swissimage 10](https://www.swisstopo.admin.ch/en/orthoimage-swissimage-10)
- **Point Cloud**: [SwissSURFACE3D](https://www.swisstopo.admin.ch/en/height-model-swisssurface3d)

## Generated Files

When you run the script, the following files will be generated:

- `mesh.dae`: Collada file containing the 3D model.
- `textura_view.jpeg`: Textured view of the 3D model.
- `textura_semantic.jpeg`: Semantic view of the 3D model's texture.

## How to Run

You can run the script from the command line as follows:

```bash
python gera_mapa.py tif_image las_file path_out
```

```bash
python gera_mapa.py tif_image las_file path_out x0 y0 size
```

## How to View

To view the 3D map, rename one of the images of your choice to texture.jpeg and open it with your preferred image viewer to see the desired information.

