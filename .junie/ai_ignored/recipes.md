# Prompt Instruction Recipes

Is AI ignored

```markdown

# Tasks

1. Analyze codebase thoroughly to understand what it is about.
2. Considering TODO, update if relevant the `README.md` and documentations in `documentation/` directory.
3. Review all documentations:
   1. Review the `README.md` updated version and make sure it aligns with the codebase;
   2. Review the newly created documentation and make sure it aligns with the codebase;
   3. If not, repeat from **Task step 2** and keep repeating until all condition are met.

```

```markdown
# First time Junie run 
Study the guidelines at `.junie/guidelines.md` thoroughly.
Provide a comprehensive summary of all guidelines studied.

# Evaluate codebase
Given your study of the guidelines at `.junie/guidelines.md`, analyse the super project and make recommendations if relevant.   
```

```markdown
# Tasks
1. Review guidelines at `.junie/guidelines.md`;
2. TODO

# Instructions
- Execute all tests before submitting.
```

## General

```markdown
Read and implement the plan at `.junie/active_plans/TODO.md`.
```

```markdown
QUESTION
Be thorough in your search.
Consider the technical implication for python, c++, ROS2 and shell scripting development.
```

## Add `dna` command

```markdown
# Tasks
1. Review guidelines at `.junie/guidelines.md`.
2. Implement dna command `dna COMMAND` in `src/lib/commands/COMMAND.bash` with the following features:
   - logic for blablabla; 
   - implement a `-F|--FLAG` flag to blablabla;
   - default case: 
     - blablab. 
   Inspire yourself with `src/lib/commands/OTHER_COMMAND.bash` for the implementation.
   Implement bats tests for `src/lib/commands/COMMAND.bash`.
   Create at least one test case per cli options.
   Inspire yourself with `tests/tests_bats/test_OTHER_COMMAND.bats`.
   Implement Markdown documentation at `documentation/command/COMMAND.md`.
4. Check if any Markdown documentation at `documentation/` need to be updated.

# Instructions
- Execute all unit-tests and all integration tests before submitting.
```
