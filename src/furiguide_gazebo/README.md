# furiguide_gazebo
gazebo simluation files for FURIguide

## make_tag_model.sh
This script autogenerates gazebo models for AprilTags. It requires imagemagick (or one can manually resize the tag images). 

### Usage
* Extract a directory of tag image PNGs (available from https://april.eecs.umich.edu/software/apriltag.html)
* run ``./make_tag_model.sh [tag number N]``
* Tag model files will be generated under models/TagN
