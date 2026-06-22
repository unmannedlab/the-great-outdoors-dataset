---
layout: default
title: Annotation
permalink: /annotation/
---

<section class="page-hero">
  <div class="wrap page-hero__grid">
    <div>
      <p class="eyebrow">Semantic annotation</p>
      <h1>An ontology for off-road scene understanding.</h1>
      <p>The annotation taxonomy extends the foundation of the RELLIS-3D dataset with terrain and object classes that appear frequently in the Great Outdoors sequences, including gravel and mulch.</p>
      <a class="button" href="{{ '/images/go_semantic.svg' | relative_url }}">Open Ontology Image</a>
    </div>
    <figure class="image-frame">
      <img src="{{ '/images/go_semantic.svg' | relative_url }}" alt="Great Outdoors Dataset ontology">
      <figcaption class="caption">Ontology definition for dataset labels.</figcaption>
    </figure>
  </div>
</section>

<section class="section">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Label set</p>
        <h2>Twenty-two classes for terrain, objects, and void regions.</h2>
      </div>
      <p>The classes cover vegetation, traversable and non-traversable terrain, structures, agents, vehicles, obstacles, and unlabeled areas for semantic segmentation research.</p>
    </div>
    <ul class="class-grid" aria-label="Semantic classes">
      <li>Void</li>
      <li>Dirt</li>
      <li>Grass</li>
      <li>Trees</li>
      <li>Pole</li>
      <li>Water</li>
      <li>Sky</li>
      <li>Vehicle</li>
      <li>Object</li>
      <li>Asphalt</li>
      <li>Building</li>
      <li>Log</li>
      <li>Person</li>
      <li>Fence</li>
      <li>Bush</li>
      <li>Concrete</li>
      <li>Barrier</li>
      <li>Puddle</li>
      <li>Mud</li>
      <li>Rubble</li>
      <li>Mulch</li>
      <li>Gravel</li>
      <li>Snow</li>
    </ul>
  </div>
</section>

<section class="section">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Caption annotation</p>
        <h2>Human-refined language labels for multimodal learning.</h2>
      </div>
      <p>Captions are produced with a semi-automatic human-in-the-loop workflow designed to scale language annotation while keeping descriptions grounded in the visual data.</p>
    </div>
    <div class="feature-grid">
      <div class="card feature">
        <div class="feature-accent"></div>
        <h3>Representative frames</h3>
        <p>Dense image sequences are downsampled with CLIP-based visual features to select diverse frames across scene conditions.</p>
      </div>
      <div class="card feature">
        <div class="feature-accent"></div>
        <h3>Model drafts</h3>
        <p>Qwen2.5-72B generates initial short and long captions that summarize scenes, terrain, objects, and spatial context.</p>
      </div>
      <div class="card feature">
        <div class="feature-accent"></div>
        <h3>Human refinement</h3>
        <p>Annotators correct errors, reduce hallucinations, and standardize language so the final captions support vision-language and multimodal learning tasks.</p>
      </div>
    </div>
  </div>
</section>

<section class="section section--paper">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Annotation examples</p>
        <h2>Representative RGB, LWIR, and language annotations.</h2>
      </div>
      <p>This sample shows the paired visual annotations used in the release: semantic labels, RGB overlays, projected LWIR overlays, and human-refined captions.</p>
    </div>
    <div class="annotation-panel-grid">
      <figure class="image-frame">
        <img src="{{ '/images/annotation_examples/annotation_rgb_original.jpg' | relative_url }}" alt="Original RGB annotation sample">
        <figcaption class="caption">Original RGB image from a snowy off-road sequence near a utility building.</figcaption>
      </figure>
      <figure class="image-frame">
        <img src="{{ '/images/annotation_examples/annotation_rgb_label.png' | relative_url }}" alt="Semantic segmentation label map for RGB sample">
        <figcaption class="caption">Semantic segmentation label map.</figcaption>
      </figure>
      <figure class="image-frame">
        <img src="{{ '/images/annotation_examples/annotation_rgb_overlay.jpg' | relative_url }}" alt="Semantic segmentation overlay on RGB sample">
        <figcaption class="caption">RGB image with semantic labels overlaid.</figcaption>
      </figure>
      <figure class="image-frame">
        <img src="{{ '/images/annotation_examples/annotation_lwir_original.jpg' | relative_url }}" alt="LWIR annotation sample">
        <figcaption class="caption">Paired LWIR image.</figcaption>
      </figure>
      <figure class="image-frame">
        <img src="{{ '/images/annotation_examples/annotation_lwir_overlay.jpg' | relative_url }}" alt="Projected semantic annotation overlay on LWIR sample">
        <figcaption class="caption">Projected semantic labels overlaid on the LWIR image.</figcaption>
      </figure>
      <div class="card caption-example">
        <span class="tag">Caption example</span>
        <h3>Short caption</h3>
        <p>Snow-covered trail with tire tracks leads toward a garage, flanked by bare trees and a utility pole under a clear sky.</p>
        <h3>Long caption</h3>
        <p>The scene shows uneven, snow-covered off-road terrain with tire tracks curving toward a gabled building with garage doors. A utility pole and leafless trees border the area, while the open snow field remains navigable with obstacle avoidance around the structure and pole.</p>
      </div>
    </div>
  </div>
</section>

<section class="section section--paper">
  <div class="wrap">
    <div class="section-head">
      <div>
        <p class="eyebrow">Statistics</p>
        <h2>Class distribution and annotation examples.</h2>
      </div>
      <p>These visual summaries help researchers understand the semantic distribution before training or evaluating models.</p>
    </div>
    <div class="figure-grid">
      <figure class="image-frame">
        <img src="{{ '/images/annotation_availability.png' | relative_url }}" alt="Annotation availability statistics">
        <figcaption class="caption">Annotation availability statistics.</figcaption>
      </figure>
      <figure class="image-frame">
        <img src="{{ '/images/class_distribution_image_count.png' | relative_url }}" alt="Image class distribution">
        <figcaption class="caption">Image label distribution.</figcaption>
      </figure>
      <figure class="image-frame">
        <img src="{{ '/images/class_distribution_pixel_count.png' | relative_url }}" alt="Pixel class distribution">
        <figcaption class="caption">Pixel label distribution.</figcaption>
      </figure>
      <figure class="image-frame">
        <img src="{{ '/images/caption_coverage.png' | relative_url }}" alt="Caption annotation coverage by split">
        <figcaption class="caption">Caption annotation coverage by split.</figcaption>
      </figure>
    </div>
  </div>
</section>
