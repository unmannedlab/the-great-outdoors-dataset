---
layout: default
title: Download
permalink: /download/
---

<section class="page-hero">
  <div class="wrap page-hero__grid">
    <div>
      <p class="eyebrow">Dataset access</p>
      <h1>Download synchronized sensor data and annotations.</h1>
      <p>Use the links below to access current annotation packages, raw ROS bags, trajectory previews, and the ontology definition.</p>
    </div>
  </div>
</section>

<section class="section section--notice">
  <div class="wrap">
    <div class="notice">
      <strong>Known issue:</strong> The 2024 dataset contains partial LiDAR data loss.
    </div>
  </div>
</section>

<section class="section">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Bulk access</p>
        <h2>Download manifest.</h2>
      </div>
      <p>Use the JSON manifest to script downloads across raw ROS bags, radar packages, and annotations.</p>
    </div>
    <div class="download-grid download-grid--compact">
      <article class="card download-card">
        <span class="tag">Manifest</span>
        <h3>All Download Links</h3>
        <p>JSON manifest with raw bag, radar, and annotation links for scripted downloads.</p>
        <a class="button" href="{{ '/google_link.json' | relative_url }}" download>Download JSON</a>
      </article>
    </div>
  </div>
</section>

<section class="section section--paper">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Curated downloads</p>
        <h2>Modality-specific packages.</h2>
      </div>
      <p>Download only the released annotation package you need, then use the raw ROS bag table for full sensor replay.</p>
    </div>
    <div class="download-grid download-grid--compact">
      <article class="card download-card">
        <span class="tag">Ontology</span>
        <h3>Ontology Definition</h3>
        <p>Semantic class ontology for interpreting labels and semantic categories.</p>
        <a class="button" href="{{ '/images/go_semantic.svg' | relative_url }}">Open Ontology</a>
      </article>
      <article class="card download-card">
        <span class="tag">RGB labels</span>
        <h3>RGB Segmentation</h3>
        <p>RGB semantic segmentation package for the released image set.</p>
        <a class="button" href="https://drive.google.com/file/d/1S5kNhSlOlikyaN6_ooWUGdD0YK7sVE9l/view?usp=drive_link">Download</a>
      </article>
      <article class="card download-card">
        <span class="tag">Captions</span>
        <h3>RGB Captions</h3>
        <p>Caption annotations for RGB frames.</p>
        <a class="button" href="https://drive.google.com/file/d/1XbGxvHxoV6I4qJMAUL7cqrG7PSeElrVH/view?usp=drive_link">Download</a>
      </article>
      <article class="card download-card">
        <span class="tag">Thermal labels</span>
        <h3>LWIR Segmentation</h3>
        <p>Long-wave infrared semantic segmentation package.</p>
        <a class="button" href="https://drive.google.com/file/d/1SM6nif5QKQOTzdNrRVPSvRNV97a_tkl-/view?usp=drive_link">Download</a>
      </article>
    </div>
  </div>
</section>

<section class="section">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Raw sequences</p>
        <h2>ROS bag downloads with trajectory previews.</h2>
      </div>
      <p>Each bag contains the original synchronized sensor topics. Use the trajectory maps to select the route before downloading the raw data.</p>
    </div>

    <div class="trajectory-overview">
      <figure class="image-frame trajectory-overview__map">
        <a href="{{ '/images/gps_trajectory_maps/combined/all_bags_trajectory.html' | relative_url }}">
          <img src="{{ '/images/gps_trajectory_maps/combined/all_bags_trajectory.png' | relative_url }}" alt="Combined GPS trajectories for all raw ROS bags">
        </a>
        <figcaption class="caption">Combined trajectory map for all recorded bags.</figcaption>
      </figure>
      <div class="trajectory-overview__stats">
        <div class="card metric">
          <strong>{{ site.data.trajectories | size }}</strong>
          <span>trajectory maps</span>
        </div>
        <div class="card metric">
          <strong>34.18 km</strong>
          <span>total recorded route length</span>
        </div>
        <div class="card metric">
          <strong>0</strong>
          <span>flagged GPS trajectory problems</span>
        </div>
        <a class="button" href="{{ '/images/gps_trajectory_maps/combined/all_bags_trajectory.html' | relative_url }}">Open Interactive Map</a>
      </div>
    </div>

    <div class="trajectory-year">
      <div class="trajectory-year__head">
        <h3>2025 ROS Bags</h3>
        <p>January 2025 collection sequences.</p>
      </div>
      <div class="trajectory-grid">
        {% for trajectory in site.data.trajectories %}
        {% if trajectory.name contains '2025-' %}
      <article class="card trajectory-card">
        <a class="trajectory-card__map" href="{{ trajectory.map_html | relative_url }}" aria-label="Open interactive trajectory map for {{ trajectory.name }}">
          <img src="{{ trajectory.map_png | relative_url }}" alt="GPS trajectory map for {{ trajectory.name }}" loading="lazy">
        </a>
        <div class="trajectory-card__body">
          <span class="tag">ROS bag</span>
          <h3>{{ trajectory.name }}</h3>
          <dl class="trajectory-meta">
            <div>
              <dt>Length</dt>
              <dd>{{ trajectory.length_km }} km</dd>
            </div>
            <div>
              <dt>Duration</dt>
              <dd>{{ trajectory.duration_min }} min</dd>
            </div>
            <div>
              <dt>GPS points</dt>
              <dd>{{ trajectory.point_count }}</dd>
            </div>
          </dl>
          <div class="button-row">
            {% if trajectory.download_url %}
            <a class="button" href="{{ trajectory.download_url }}">Download Bag</a>
            {% else %}
            <span class="button button--disabled" aria-disabled="true">No Bag Link</span>
            {% endif %}
            {% if trajectory.radar_download_url %}
            <a class="button button--quiet" href="{{ trajectory.radar_download_url }}">Download Radar</a>
            {% endif %}
            <a class="button button--quiet" href="{{ trajectory.map_html | relative_url }}">Open Map</a>
          </div>
        </div>
      </article>
        {% endif %}
        {% endfor %}
      </div>
    </div>

    <div class="trajectory-year">
      <div class="trajectory-year__head">
        <h3>2024 ROS Bags</h3>
        <p>April 2024 collection sequences. Known issue: The 2024 dataset contains partial LiDAR data loss.</p>
      </div>
      <div class="trajectory-grid">
        {% for trajectory in site.data.trajectories %}
        {% if trajectory.name contains '2024-' %}
      <article class="card trajectory-card">
        <a class="trajectory-card__map" href="{{ trajectory.map_html | relative_url }}" aria-label="Open interactive trajectory map for {{ trajectory.name }}">
          <img src="{{ trajectory.map_png | relative_url }}" alt="GPS trajectory map for {{ trajectory.name }}" loading="lazy">
        </a>
        <div class="trajectory-card__body">
          <span class="tag">ROS bag</span>
          <h3>{{ trajectory.name }}</h3>
          <dl class="trajectory-meta">
            <div>
              <dt>Length</dt>
              <dd>{{ trajectory.length_km }} km</dd>
            </div>
            <div>
              <dt>Duration</dt>
              <dd>{{ trajectory.duration_min }} min</dd>
            </div>
            <div>
              <dt>GPS points</dt>
              <dd>{{ trajectory.point_count }}</dd>
            </div>
          </dl>
          <div class="button-row">
            {% if trajectory.download_url %}
            <a class="button" href="{{ trajectory.download_url }}">Download Bag</a>
            {% else %}
            <span class="button button--disabled" aria-disabled="true">No Bag Link</span>
            {% endif %}
            {% if trajectory.radar_download_url %}
            <a class="button button--quiet" href="{{ trajectory.radar_download_url }}">Download Radar</a>
            {% endif %}
            <a class="button button--quiet" href="{{ trajectory.map_html | relative_url }}">Open Map</a>
          </div>
        </div>
      </article>
        {% endif %}
        {% endfor %}
      </div>
    </div>
  </div>
</section>

<section class="section">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">ROS topics</p>
        <h2>Raw bag contents.</h2>
      </div>
      <p>The raw ROS bag includes navigation, imaging, radar, LiDAR, command, and transform topics.</p>
    </div>
    <div class="table-wrap">
      <table>
        <thead>
          <tr>
            <th>Topic name</th>
            <th>Message type</th>
            <th>Description</th>
          </tr>
        </thead>
        <tbody>
          <tr><td>/Navtech/FFTData</td><td>nav_ross/HighPrecisionFFTData</td><td>Radar FFT data</td></tr>
          <tr><td>/lester/imu/data</td><td>sensor_msgs/Imu</td><td>Filtered IMU data from the embedded Warthog IMU</td></tr>
          <tr><td>/lester/imu/data_raw</td><td>sensor_msgs/Imu</td><td>Raw IMU data from the embedded Warthog IMU</td></tr>
          <tr><td>/img_node/intensity_image</td><td>sensor_msgs/Image</td><td>Intensity image generated by the Ouster LiDAR</td></tr>
          <tr><td>/lester/imu/mag</td><td>sensor_msgs/MagneticField</td><td>Raw magnetic field data from the embedded Warthog IMU</td></tr>
          <tr><td>/lester/lidar_points</td><td>sensor_msgs/PointCloud2</td><td>Point cloud data from the Ouster LiDAR</td></tr>
          <tr><td>/lester/ouster_center/imu</td><td>sensor_msgs/Imu</td><td>Raw IMU data from the embedded Ouster LiDAR IMU</td></tr>
          <tr><td>/lester/lwir_front/camera_info</td><td>sensor_msgs/CameraInfo</td><td>Intrinsics of the thermal camera</td></tr>
          <tr><td>/lester/lwir_front/image_rect/compressed</td><td>sensor_msgs/CompressedImage</td><td>Compressed thermal image stream</td></tr>
          <tr><td>/lester/stereo_left/camera_info</td><td>sensor_msgs/CameraInfo</td><td>Left RGB camera intrinsics</td></tr>
          <tr><td>/lester/stereo_left/image_rect_color/compressed</td><td>sensor_msgs/CompressedImage</td><td>Compressed image stream from the left RGB camera</td></tr>
          <tr><td>/lester/stereo_right/camera_info</td><td>sensor_msgs/CameraInfo</td><td>Right RGB camera intrinsics</td></tr>
          <tr><td>/lester/stereo_right/image_rect_color/compressed</td><td>sensor_msgs/CompressedImage</td><td>Compressed image stream from the right RGB camera</td></tr>
          <tr><td>/lester/rear_center/camera_info</td><td>sensor_msgs/CameraInfo</td><td>Rear center RGB camera intrinsics</td></tr>
          <tr><td>/lester/rear_center/image_rect_color/compressed</td><td>sensor_msgs/CompressedImage</td><td>Compressed image stream from the rear RGB camera</td></tr>
          <tr><td>/lester/ublox/fix</td><td>sensor_msgs/NavSatFix</td><td>INS data from u-blox</td></tr>
          <tr><td>/lester/right_drive/status/battery_current</td><td>std_msgs/Float64</td><td>Right drive battery current</td></tr>
          <tr><td>/lester/right_drive/status/battery_voltage</td><td>std_msgs/Float64</td><td>Right drive battery voltage</td></tr>
          <tr><td>/lester/left_drive/status/battery_current</td><td>std_msgs/Float64</td><td>Left drive battery current</td></tr>
          <tr><td>/lester/left_drive/status/battery_voltage</td><td>std_msgs/Float64</td><td>Left drive battery voltage</td></tr>
          <tr><td>/lester/rc_teleop/cmd_vel</td><td>geometry_msgs/Twist</td><td>RC input command to the Warthog</td></tr>
          <tr><td>/tf</td><td>tf2_msgs/TFMessage</td><td>Dynamic transforms</td></tr>
          <tr><td>/tf_static</td><td>tf2_msgs/TFMessage</td><td>Static transforms</td></tr>
        </tbody>
      </table>
    </div>
  </div>
</section>
