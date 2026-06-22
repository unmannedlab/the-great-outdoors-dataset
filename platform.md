---
layout: default
title: Platform
permalink: /platform/
---

<section class="page-hero">
  <div class="wrap page-hero__grid">
    <div>
      <p class="eyebrow">Recording platform</p>
      <h1>Field data captured from an off-road UGV.</h1>
      <p>The dataset was collected using a Clearpath Robotics Warthog equipped with LiDAR, RGB cameras, thermal imaging, radar, inertial sensing, and RTK GPS for synchronized multimodal perception in unstructured terrain.</p>
      <a class="button" href="https://clearpathrobotics.com/warthog-unmanned-ground-vehicle-robot/">View Warthog Platform</a>
    </div>
    <figure class="image-frame">
      <img src="{{ '/images/3D_arl.png' | relative_url }}" alt="3D scan of the Great Outdoors Dataset sensor setup">
      <figcaption class="caption">3D scan of the sensor setup.</figcaption>
    </figure>
  </div>
</section>

<section class="section">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Sensor setup</p>
        <h2>Complementary sensing for difficult outdoor conditions.</h2>
      </div>
      <p>Each sensor stream contributes a different view of the environment, from dense geometry and visual appearance to thermal contrast, radar returns, and precise vehicle motion.</p>
    </div>
    <div class="sensor-grid">
      <div class="card sensor-card">
        <h3>64-channel LiDAR</h3>
        <p><a href="https://ouster.com/products/os1-lidar-sensor">Ouster OS1</a> point clouds for 3D geometry and semantic segmentation.</p>
      </div>
      <div class="card sensor-card">
        <h3>RGB cameras</h3>
        <p>Three <a href="https://www.baslerweb.com/en/products/cameras/area-scan-cameras/ace/aca1920-50gc/">Basler acA1920-50gc</a> cameras with Edmund Optics 16mm lenses.</p>
      </div>
      <div class="card sensor-card">
        <h3>Thermal camera</h3>
        <p><a href="https://www.flir.com/products/boson/?model=20640A032&amp;vertical=lwir&amp;segment=oem">FLIR Boson 640</a> long-wave infrared imagery for low-visibility conditions.</p>
      </div>
      <div class="card sensor-card">
        <h3>INS and GPS</h3>
        <p><a href="http://www.microstrain.com/inertial-sensors/3dm-gx5-25">MicroStrain 3DM-GX5-AHRS</a> for motion and orientation measurements.</p>
      </div>
      <div class="card sensor-card">
        <h3>2D mmWave radar</h3>
        <p><a href="https://navtechradar.com/radar-solutions/radar-for-intelligent-transport-solutions/clearway-technical-specification/">Navtech CTS350-X</a> radar data for robust perception in adverse conditions.</p>
      </div>
      <div class="card sensor-card">
        <h3>RTK GPS</h3>
        <p><a href="https://www.sparkfun.com/products/19984">SparkFun RTK Facet</a> positioning for precise geolocation.</p>
      </div>
    </div>
  </div>
</section>

<section class="section section--paper">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Platform imagery</p>
        <h2>Vehicle and sensor views.</h2>
      </div>
    </div>
    <div class="media-pair">
      <figure class="image-frame">
        <img src="{{ '/images/sensor_setup.png' | relative_url }}" alt="Sensor layout illustration">
        <figcaption class="caption">Sensor setup illustration.</figcaption>
      </figure>
      <figure class="image-frame">
        <img src="{{ '/images/on_field_image.jpg' | relative_url }}" alt="Warthog platform in the field">
        <figcaption class="caption">Warthog platform during field collection.</figcaption>
      </figure>
    </div>
  </div>
</section>
