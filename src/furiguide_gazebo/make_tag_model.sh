#!/bin/sh

TAG=Tag$1
DESTDIR="models/"
TAGDIR="$DESTDIR/${TAG}/"

TAG_IMAGES_DIR=tag36h11/
TAG_IMAGE="tag36_11_$(printf '%05d' $1).png"

# initialize the models directory structure
mkdir -vp "${TAGDIR}"
mkdir -vp "${TAGDIR}materials/scripts/"
mkdir -vp "${TAGDIR}materials/textures/"

# copy and resize the appropriate tag image
convert -verbose -scale 1000x1000 "$TAG_IMAGES_DIR/$TAG_IMAGE" "${TAGDIR}/materials/textures/${TAG_IMAGE}"

# create boilerplate files
cat << EOF > ${TAGDIR}/model.config
<?xml version="1.0" ?>
<model>
    <name>${TAG}</name>
    <version>1.0</version>
    <sdf version="1.6">model.sdf</sdf>
    <author>
        <name></name>
        <email></email>
    </author>
    <description></description>
</model>
EOF

cat << EOF > ${TAGDIR}/model.sdf
<?xml version='1.0'?>                                                                                                                                                                                                                                                                                                                                                                                                                   
<sdf version='1.6'>
  <model name='${TAG}'>
    <static>true</static>
    <link name='link'>
      <pose frame=''>0 0 0 0 0 0</pose>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>0</gravity>
      <visual name='visual'>
        <pose frame=''>0 0 0 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <box>
            <size>0.1615 0.001 0.1615</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>model://${TAG}/materials/scripts</uri>
            <uri>model://${TAG}/materials/textures</uri>
            <name>vrc/${TAG}</name>
          </script>
        </material>
      </visual>
      <collision name='collision'>
        <pose frame=''>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.1615 0.001 0.1615</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF

cat << EOF > ${TAGDIR}/materials/scripts/${TAG}.material
material vrc/${TAG}
{
        receive_shadows off
        technique
        {
                pass
                {
                        texture_unit
                        {
                                texture ${TAG_IMAGE}
                                filtering anistropic
                                max_anisotropy 16
                                scale 1 1
                        }
                }
        }
}
EOF

