# RoboSub Software and Documentation Hub

This site contains tool setup instructions, software guides, and firmware
documentation for RoboSub.

## Contributing to docs

### Editing an existing page

Open `RoboSub` in VSCode and modify the corresponding Markdown file in `docs/`
and save. Everything is written in Markdown, see this [guide](https://www.markdownguide.org/basic-syntax/)
for syntax.

### Add or re-organize a page

Open `RoboSub` in VSCode and open the `mkdocs.yml`. If you want a new page
(e.g. like the existing `Mechanical`, `Firmware`), do the following

```yaml
nav:
   - Home: index.md

   - General tools:
     ....
   - My New page:
```

and if you want a new section, add it to the page

```yaml
nav:
   - Home: index.md

   - General tools:
     ....
   - My New page:
      - My new section: test/test-section.md
```

### Preview the website

From the `RoboSub` root, run the script for your operating system.

Windows (PowerShell or Command Prompt):

```powershell
.\preview-docs.cmd
```

macOS or Linux:

```sh
sh preview-docs.sh
```

Open <http://127.0.0.1:8000/> and leave the terminal running. Save changes to
`docs/` or `mkdocs.yml` to rebuild the site and refresh the browser
automatically. 

If port 8000 is busy, use a different port:

```powershell
.\preview-docs.cmd --dev-addr 127.0.0.1:8001
```

```sh
sh preview-docs.sh --dev-addr 127.0.0.1:8001
```

Then open <http://127.0.0.1:8001/>. If installation or a build fails, check the
terminal output, fix the issue, and run the same script again.

