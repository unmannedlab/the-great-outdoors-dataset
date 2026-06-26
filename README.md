# god_dataset_site
"""
Install [jekyll](https://jekyllrb.com/)

bundle config set path "vendor/bundle"

bundle install

bundle exec jekyll serve --no-watch"

## Local preview

Use `bundle exec jekyll serve --no-watch` if Linux reports `iNotify max watches exceeded`. The site will build and serve normally, but you must restart the command after edits.

For live auto-regeneration, increase the system watch limit:

```bash
sudo sysctl fs.inotify.max_user_watches=524288
echo fs.inotify.max_user_watches=524288 | sudo tee /etc/sysctl.d/99-jekyll-watch.conf
sudo sysctl --system
```


## Visitor analytics

This Jekyll site includes a visitor counter on the homepage and footer. GitHub Pages is static, so live visit counts must come from a public third-party counter service configured through `_data/analytics.yml`.

### Service used

The site is currently configured to use Busuanzi, a lightweight no-account visitor counter for static sites. When enabled, the layout loads the public Busuanzi script and displays total site visits plus visitor count. No private API key or token is stored in the frontend.

GoatCounter support is still present in the layout if you prefer to use a dedicated analytics account later.

### Setup

Current no-account setup:

```yml
enabled: true
provider: busuanzi
busuanzi_script_url: "https://busuanzi.ibruce.info/busuanzi/2.3/busuanzi.pure.mini.js"
```

Optional GoatCounter setup:

```yml
enabled: true
provider: goatcounter
goatcounter_code: "great-outdoors-dataset"
counter_path: "TOTAL"
dashboard_url: "https://great-outdoors-dataset.goatcounter.com"
```

If you want country or region rows to appear directly on the homepage, provide a public aggregate JSON endpoint in `location_stats_url`. The endpoint must allow browser access with CORS. Do not use a private API endpoint that requires a secret token in browser code. The frontend accepts either of these shapes:

```json
[{ "label": "United States", "value": 120 }]
```

```json
{ "countries": [{ "country": "United States", "visits": 120 }] }
```

Rebuild locally with `bundle exec jekyll build` before deploying.

### Privacy notes

The site only supports public aggregate counters and public aggregate location summaries. It does not store analytics data in GitHub Pages and it does not include private API keys, tokens, or precise individual visitor locations in the frontend. Location reporting depends on the analytics provider's IP geolocation and should be treated as approximate country/region-level information.

