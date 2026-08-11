# Guidelines 

This page specifies guidelines for developing and deploying code.

## Development

### Naming convention

For embedded development, follow the [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/naming.html)

In summary:

| Item | Convention |
| --- | --- |
| Modules | `snake_case` |
| Types | `UpperCamelCase` |
| Functions | `snake_case` |
| Methods | `snake_case` |
| Local variables | `snake_case` |
| Statics | `SCREAMING_SNAKE_CASE` |
| Constants | `SCREAMING_SNAKE_CASE` |

### Line width

Keep all line width to 100. This is checked by linters.

## Deployment

Deploy via PR's. Github actions will be ran to check code.
Merge (ideally with another contributor's approval) to main when test passes.
Do not push to main. This forbidden to prevent breaking changes.

### Commit messages

Follow [conventional commits](https://www.conventionalcommits.org/en/v1.0.0/)
for commit messages. This is enforced by the `commitlint`.

### Github Actions

Current stack:

| Action | Trigger | Purpose |
|---|---|---| 
| Documentation / Pages | Push to `main`, pull request | Builds the MkDocs documentation site and deploys it to GitHub Pages after changes reach `main`. |
| Commit Lint | Pull request / push | Validates commit messages against the repository's commit-message convention. |
| MkDocs Build | Pull request | Builds the documentation with `mkdocs build --strict` to catch broken documentation before merge. |
