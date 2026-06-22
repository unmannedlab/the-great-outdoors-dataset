---
layout: default
title: Home
permalink: /
---

<section class="hero">
  <div class="hero__background" aria-hidden="true">
    <img src="{{ '/images/GO_image.png' | relative_url }}" alt="">
  </div>
  <div class="hero__shade" aria-hidden="true"></div>
  <div class="wrap hero__inner">
    <p class="eyebrow">Off-road multimodal dataset</p>
    <h1>The Great Outdoors Dataset</h1>
    <p class="hero__lead">A multimodal perception dataset for autonomous navigation in unstructured outdoor terrain, combining LiDAR, RGB imagery, thermal imaging, radar, INS, and RTK GPS with semantic annotations.</p>
    <div class="hero__actions">
      <a class="button button--light" href="{{ '/download/' | relative_url }}">Download Dataset</a>
      <a class="button button--ghost" href="{{ '/annotation/' | relative_url }}">Explore Ontology</a>
    </div>
    <div class="hero__stats" aria-label="Dataset highlights">
      <div class="stat">
        <strong>64</strong>
        <span>channel Ouster LiDAR</span>
      </div>
      <div class="stat">
        <strong>3</strong>
        <span>RGB camera streams</span>
      </div>
      <div class="stat">
        <strong>23</strong>
        <span>semantic classes</span>
      </div>
      <div class="stat">
        <strong>16</strong>
        <span>raw ROS bag trajectories</span>
      </div>
    </div>
  </div>
</section>

<section class="partner-band">
  <div class="wrap partner-grid">
    <p>Developed by Texas A&amp;M University and DEVCOM Army Research Laboratory</p>
    <div class="partner-logos">
      <a href="https://www.tamu.edu/">
        <img src="{{ '/images/tamu_logo.png' | relative_url }}" alt="Texas A&amp;M University">
      </a>
      <a href="https://www.arl.army.mil/">
        <img src="{{ '/images/arl_logo.png' | relative_url }}" alt="DEVCOM Army Research Laboratory">
      </a>
    </div>
  </div>
</section>

<section class="section">
  <div class="wrap split">
    <div>
      <p class="eyebrow">Research focus</p>
      <h2>Built for perception beyond paved roads.</h2>
      <p class="lead">The Great Outdoors Dataset supports robust autonomy research in complex off-road environments where terrain, lighting, vegetation, and surface conditions vary constantly.</p>
      <p>Collected with an unmanned ground vehicle designed for unstructured terrain, the dataset brings together high-density 3D point clouds, high-resolution RGB imagery, long-wave infrared imaging, radar, inertial navigation, and precise geolocation. It extends the foundation of <a href="https://github.com/unmannedlab/RELLIS-3D">RELLIS-3D</a> with additional terrain and object categories for outdoor autonomy.</p>
    </div>
    <div class="metric-grid">
      <div class="card metric">
        <strong>LiDAR</strong>
        <span>Detailed point clouds for geometry, mapping, and 3D semantic segmentation.</span>
      </div>
      <div class="card metric">
        <strong>RGB</strong>
        <span>Multiple color camera views for visual perception and sensor fusion.</span>
      </div>
      <div class="card metric">
        <strong>LWIR</strong>
        <span>Thermal imagery for low-light and adverse visibility conditions.</span>
      </div>
      <div class="card metric">
        <strong>Radar</strong>
        <span>2D mmWave radar data for complementary perception in challenging weather.</span>
      </div>
    </div>
  </div>
</section>

<section class="section section--paper">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Dataset preview</p>
        <h2>Multimodal scenes with semantic structure.</h2>
      </div>
      <p>The data is tailored for machine learning models that need to understand vegetation, trails, mud, rubble, water, barriers, vehicles, and other off-road scene elements.</p>
    </div>
    <figure class="image-frame">
      <img src="{{ '/images/dataset_overview.png' | relative_url }}" alt="Examples of synchronized data from the Great Outdoors Dataset">
      <figcaption class="caption">Example synchronized views from the dataset.</figcaption>
    </figure>
  </div>
</section>

<section class="section">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Why it matters</p>
        <h2>A practical benchmark for off-road autonomy.</h2>
      </div>
      <p>Urban datasets rarely capture the perception problems found in unstructured terrain. This dataset is organized around those harder outdoor conditions.</p>
    </div>
    <div class="feature-grid">
      <div class="card feature">
        <div class="feature-accent"></div>
        <h3>Sensor fusion</h3>
        <p>Combine LiDAR, camera, thermal, radar, INS, and RTK GPS data for multimodal perception pipelines.</p>
      </div>
      <div class="card feature">
        <div class="feature-accent"></div>
        <h3>Semantic understanding</h3>
        <p>Train and evaluate models across terrain, vegetation, people, objects, vehicles, structures, and void regions.</p>
      </div>
      <div class="card feature">
        <div class="feature-accent"></div>
        <h3>Outdoor complexity</h3>
        <p>Work with unstructured scenes that include gravel, mulch, mud, puddles, rubble, grass, trees, and water.</p>
      </div>
    </div>
  </div>
</section>

<section class="section section--paper">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Citation</p>
        <h2>Cite the dataset.</h2>
      </div>
      <p>Please cite the Great Outdoors Dataset paper if this data supports your research.</p>
    </div>
    <div class="card citation-card">
<pre>@misc{jiang2025gogreatoutdoorsmultimodal,
  title={GO: The Great Outdoors Multimodal Dataset},
  author={Peng Jiang and Kasi Viswanath and Akhil Nagariya and George Chustz and Maggie Wigness and Philip Osteen and Timothy Overbye and Christian Ellis and Long Quang and Srikanth Saripalli},
  year={2025},
  eprint={2501.19274},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2501.19274},
}</pre>
    </div>
  </div>
</section>

<section class="section section--paper">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Collaborators</p>
        <h2>Research contributors.</h2>
      </div>
    </div>
    <div class="collab-list">
      <div class="card collab">
        <h3><a href="https://www.tamu.edu/">Texas A&amp;M University</a></h3>
        <p>Peng Jiang, Kasi Viswanath, Akhil Nagariya, George Chustz, Srikanth Saripalli</p>
      </div>
      <div class="card collab">
        <h3><a href="https://www.arl.army.mil/">CCDC Army Research Laboratory</a></h3>
        <p>Maggie Wigness, Philip Osteen, Tim Overbye, Christian Ellis, Long Quang</p>
      </div>
    </div>
    <p class="license-note">All datasets and code on this page are copyrighted by the authors and published under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 License.</p>
  </div>
</section>
