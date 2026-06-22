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
    </div>
  </div>
</section>
