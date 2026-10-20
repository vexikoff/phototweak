# AGENTS.md: guide for AI agents and contributors
## 1. Golden rules

### Never crash (outranks feature work)

People trust PhotoTweak with their work, and a crash loses it. A malformed file, a bad command or MCP param, a corrupt settings file, an odd keystroke or a full disk must produce an error the user or agent can act on, never a panic. Don't ship a feature by adding a panic path; fix a crash before building on top of it.
