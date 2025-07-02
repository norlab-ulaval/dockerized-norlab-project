# Prompt Instruction Recipes

## General

Follow guidelines at `.junie/guidelines.md`.
Execute all unit-tests and all integration tests before submiting.

Add/refactor/improve TODO 
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose source code change if relevant. 
Update Markdown documentation accordingly. 

Inspire yourself with `TODO`.

## Bats unit-test generation 

Implement a bats test for `src/lib/commands/TODO.bash`.
Follow guidelines at `.junie/guidelines.md`.
Inspire yourself with `tests/tests_bats/test_TODO.bats`.
Create at least one test case per command argument and/or options.
Test relevant option and arguments combinaison.
You can mock their corresponding functions as the intended purposes of this test file is for unit-testing the CLI functionalities.
Propose source code change if relevant. 
Update Markdown documentation accordingly. 
Execute all unit-tests and all integration tests before submiting.

## Improve/refactor source code

Refactor/improve `src/lib/commands/TODO.bash`.
TODO
Follow guidelines at `.junie/guidelines.md`.
Update `test_TODO.bats` accordingly.
Create at least one test case per new command argument and/or options, update current tests cases otherwise.
Test relevant option and arguments combinaison.
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose source code change if relevant.
Update Markdown documentation accordingly. 
Execute all unit-tests and all integration tests before submiting.

## Modify proposed tests solutions 

In `TODO.bats`, instead of mocking `find`, `grep`, `cut`, `cd`, `pwd`, `command` and `basename` command, use the real one and tests the result using bats assert functionalities as instructed in `guidelines.md`

---

Integration tests `TODO` and `TODO` are all failing. 
Please investigate and make the required changes. 
Always follow guidelines at `.junie/guidelines.md`.

---

You overcomplicated `TODO` new test cases. 
Don't test flag that are not part of the cli definition even if they are mentioned in the doc.
You only need a test case for TODO

---

The following proposed code in `TODO.bash` is overcomplicated 
```shell
TODO
```
Instead, inspire yourself with `TODO.bash` cli implementation:
```shell
TODO
```
Its clearer, explicit and more intuitive.
 
