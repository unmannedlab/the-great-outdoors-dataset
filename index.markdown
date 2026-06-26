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
        <strong>17</strong>
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

{% assign analytics = site.data.analytics %}
<section class="section section--paper analytics-section" id="visitor-analytics" data-analytics-section data-location-stats-url="{{ analytics.location_stats_url | escape }}" data-location-limit="{{ analytics.location_limit | default: 6 }}">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Visitor analytics</p>
        <h2>Public aggregate traffic signals.</h2>
      </div>
      <p>Privacy-friendly counts help the community understand dataset interest without exposing precise individual visitor activity.</p>
    </div>
    <div class="analytics-grid">
      <article class="card analytics-card analytics-counter">
        <span class="tag">Visits</span>
        <h3>Total site visits</h3>
        {% if analytics.enabled and analytics.provider == 'goatcounter' and analytics.goatcounter_code and analytics.goatcounter_code != '' %}
        <img class="analytics-counter__image" src="https://{{ analytics.goatcounter_code }}.goatcounter.com/counter/{{ analytics.counter_path | default: 'TOTAL' }}.svg" alt="Total site visit counter">
        <p>Displayed using the public GoatCounter counter endpoint.</p>
        {% elsif analytics.enabled and analytics.provider == 'busuanzi' %}
        <div class="analytics-counter__numbers" aria-label="Visitor counter">
          <div>
            <strong data-counter-copy="site_pv">...</strong>
            <span>Total visits</span>
          </div>
          <div>
            <strong data-counter-copy="site_uv">...</strong>
            <span>Visitors</span>
          </div>
        </div>
        <p>Displayed using a lightweight public visitor counter for static sites.</p>
        {% else %}
        <strong class="analytics-placeholder">Setup pending</strong>
        <p>Configure <code>_data/analytics.yml</code> to show the live counter.</p>
        {% endif %}
      </article>
      <article class="card analytics-card">
        <span class="tag">Locations</span>
        <h3>Aggregate visitor regions</h3>
        <ul class="analytics-location-list" data-analytics-locations>
          {% for stat in analytics.location_stats %}
          <li>
            <span>{{ stat.label }}</span>
            <strong>{{ stat.value }}</strong>
            {% if stat.detail %}<small>{{ stat.detail }}</small>{% endif %}
          </li>
          {% endfor %}
        </ul>
        <p class="analytics-note" data-analytics-status>Country or region statistics load only from a configured public aggregate endpoint.</p>
      </article>
      <article class="card analytics-card">
        <span class="tag">Privacy</span>
        <h3>Static-site friendly</h3>
        <p>The website does not store analytics data itself and does not include private API keys. Configure only public service identifiers or public aggregate endpoints.</p>
        {% if analytics.dashboard_url and analytics.dashboard_url != '' %}
        <a class="button button--quiet" href="{{ analytics.dashboard_url }}">Open Analytics Dashboard</a>
        {% else %}
        <p class="analytics-note">Add a public dashboard URL in <code>_data/analytics.yml</code> after enabling shared analytics.</p>
        {% endif %}
      </article>
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
  author={Peng Jiang and Kasi Viswanath and Akhil Nagariya and George Chustz and Maggie Wigness and Philip Osteen and Timothy Overbye and Christian Ellis and Long Quang and Jia Huang and Srikanth Saripalli},
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
        <p>Peng Jiang, Kasi Viswanath, Akhil Nagariya, George Chustz, Jia Huang, Srikanth Saripalli</p>
      </div>
      <div class="card collab">
        <h3><a href="https://www.arl.army.mil/">DEVCOM Army Research Laboratory</a></h3>
        <p>Maggie Wigness, Philip Osteen, Tim Overbye, Christian Ellis, Long Quang</p>
      </div>
    </div>
    <p class="license-note">All datasets and code on this page are copyrighted by the authors and published under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 License.</p>
  </div>
</section>
