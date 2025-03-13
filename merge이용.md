## merge단계(맡은 사람)
1. dev의 최신상황 받아오기
- git switch dev 
- git pull origin dev
2. 팀원의 브랜치 최신상황 받아오기
  - git switch "팀원브랜치"
  - git pull origin "팀원브랜치"
3. 본인과 팀원의 변경사항 dev로 merge
  - git switch dev
  - git merge "자신의 브랜치"  
  - git merge "팀원브랜치"
4. merge중 상황
  - conflict(충돌 발생시)
    - conflict일어난 파일 팀원과 상의해서 업데이트
    - git add . 
    - git commit -m "docs: merge conflict가 test.txt에서 발생하여 파일 삭제(불필요한 파일)"
5. merge완료 후 확인을 위해 gerrit으로 push
  - git push origin HEAD:refs/for/dev
6. 팀원과 자신의 브랜치 삭제
  - git branch -d "자신의 브랜치"
  - git branch -d "팀원브랜치"
7. 팀원과 자신의 브랜치 새로 dev에서 받아온 이후 새로운 브랜치 적용을 위해 push(**브랜치 적용할 때만 직접 push**)- 
  - git branch "자신의 브랜치" dev
  - git push origin "자신의 브랜치"
  - git branch "팀원브랜치" dev
  - git push origin "팀원브랜치"

## merge적용(받는사람)
1. dev 업데이트 받아오기
  - git switch origin dev
  - git pull origin dev
2. 자신의 브랜치 업데이트 받아오기
  - git switch origin "자신의 브랜치"
  - git pull origin "자신의 브랜치"