import pandas as pd
import random

path = '/home/seonwoo/Desktop/workspace/Banking_System_Algorithm'  # 기본경로(폴더)

user_df = pd.read_csv(path + '/user_csv', sep=',',
                      encoding='cp949')  # 작업 데이터프레임

if user_df.empty == True:  # 시작 시, 정보 출력
    print("비어있음")
else:
    print(user_df)


class Creater:  # 1번 유저 생성 클래스
    def __init__(self, user_df):
        self.user_df = user_df

    def create_name(self):
        print("-" * 40)
        print("### 유저 생성 모드입니다 ###")
        self.name = input(" >> 성함을 입력해 주십시오 : ")
        if self.name == "Exit" or self.name == "exit":
            return False  # while문은 밖의 본문에서..
        elif self.name in self.user_df["Name"].tolist():
            print("중복된 이름입니다. 다시입력하여 주십시오.")
            return False

    def create_id(self):
        self.user_id = input(" >> 생성하실 ID를 입력해 주십시오 : ")
        # self.user_df = pd.read_csv(path + '/user_csv', sep=',',
        #                            encoding='cp949')
        if self.user_id == "Exit" or self.user_id == "exit":
            return False
        if self.user_id in self.user_df.ID.tolist():
            print("중복된 아이디 입니다. 다시 입력하여 주십시오.")
            return False
        elif len(self.user_id) >= 4 and self.user_id.isalnum() == True:
            self.good_id = self.user_id
        else:
            cnt = 1
            while True:
                if len(self.user_id) >= 4 and self.user_id.isalnum() == True:
                    self.good_id = self.user_id
                    response = True
                    break
                if cnt == 3:
                    print("3회이상 잘못된 ID를 입력하였습니다. 유저 생성모드를 종료합니다.")
                    response = False
                    break
                elif self.user_id.isalnum() == False and len(self.user_id) < 4:
                    self.user_id = input(
                        " >> 아이디 형식에 적합하지 않으며 길이가 짧습니다. 다시 입력하여 주십시오 : ")
                    cnt += 1
                    if self.user_id == "Exit" or self.user_id == "exit":
                        response = False
                        break
                elif len(self.user_id) < 4:
                    self.user_id = input(
                        " >> 아이디의 길이가 너무 짧습니다. 다시입력하여 주십시오(4자리 이상) : ")
                    cnt += 1
                    if self.user_id == "Exit" or self.user_id == "exit":
                        response = False
                        break
            if response == False:  # 이 전 단계로 돌아가기
                return False
            elif response == True:
                return True

    def create_pw(self):
        while True:
            self.passward = input(" >> 생성하실 비밀번호를 입력해 주십시오 : ")
            if self.passward == "Exit" or self.passward == "exit":
                response = False
                break
            if len(self.passward) < 8 or self.passward.isdigit() == True or self.passward.isalpha() == True or self.passward.isalnum() == True or self.passward == self.passward.lower():
                print("잘못된 비밀번호 형식입니다.")
                continue
            else:
                self.good_pw = self.passward
                good_pw_again = input(" >> 비밀번호를 한번 더 입력해 주십시오 :  ")
                if good_pw_again == self.good_pw:
                    response = True
                    break
                elif good_pw_again == "Exit" or good_pw_again == "exit":
                    response = False
                    continue
                else:
                    print("올바른 암호가 아닙니다. 비밀번호 초기설정으로 돌아갑니다.")
                    continue
        if response == False:
            return False
        elif response == True:
            return True

    def create_acc(self):
        num_list = []
        for i in range(8):
            num_list.append(random.randint(0, 9))
        self.account = "".join(map(str, num_list))
        self.seed = 100000

    def create_df(self):
        user = {"Name": self.name, "ID": self.good_id,
                "PW": self.good_pw, "Account": self.account, "Balance": self.seed}

        self.user_df = pd.concat([self.user_df, pd.DataFrame(
            user, index=[0])], ignore_index=True)
        self.user_df.to_csv(path + '/user_csv',
                            index=False, encoding='cp949')  # 변경내용 저장
        print("생성되었습니다.")


class Remittance(Creater):  # 3번 랜덤 송금 클래스
    def __init__(self, user_df):
        Creater.__init__(self, user_df)

    def remit(self):
        print("-" * 40)
        print("### 랜덤 계좌 송금 모드입니다 ###")
        balance = self.user_df.Balance.copy()
        if self.user_df.empty:
            print("등록된 유저가 없습니다. 유저 생성모드에서 유저를 생성하십시오")
            return False
        elif len(self.user_df) == 1:
            print("유저는 자기 자신에게 송금할 수 없습니다. 유저를 추가로 생성하여 주십시오.")
            return False
        else:
            c = 0
            while True:
                spend_index = random.randrange(0, len(balance))  # 보내는 랜덤 인덱스
                recieve_index = random.randrange(0, len(balance))  # 받는 랜덤 인덱스
                if spend_index == recieve_index:
                    continue
                else:
                    money = random.randint(0, 100000)
                    if balance[spend_index] >= money:
                        balance[spend_index] = balance[spend_index] - money
                        balance[recieve_index] = balance[recieve_index] + money
                        c += 1
                        f = open(
                            f"{path}/{self.user_df.Name[spend_index]}.txt", "a+")
                        f.write(
                            f"{self.user_df.Name[recieve_index]}님에게 {money}원을 송금하였습니다.\n")
                        f.close()
                    elif balance[spend_index] < money:
                        if c < len(balance):
                            continue
                        else:
                            print(f"송금이 {c}번 진행되었습니다. ")
                            self.user_df.Balance = balance
                            self.user_df.to_csv(path + '/user_csv',
                                                index=False, encoding='cp949')  # 변경내용 저장
                            print(
                                f"{money - balance[spend_index]}원이 부족하여 송금을 종료합니다.")
                            return False


class Killer(Creater):  # 4번 유저삭제 클래스
    def __init__(self, user_df):
        Creater.__init__(self, user_df)

    def kill(self):
        print("-" * 40)
        print("### 유저 삭제 모드입니다 ###")
        del_id = input(" >> 삭제하실 유저의 ID를 입력하여 주십시오 : ")
        if del_id == "Exit" or del_id == "exit":
            return False
        else:
            if del_id in self.user_df.ID.tolist():
                del_pw = input(" >> 삭제하실 유저의 비밀번호를 입력하여 주십시오 : ")
                if del_pw == "Exit" or del_pw == "exit":
                    return False
                else:
                    if del_pw in self.user_df.PW.tolist():
                        del_index = self.user_df.ID.tolist().index(del_id)
                        self.user_df = self.user_df.drop(index=del_index, axis="rows").reset_index(
                            drop=True)
                        print("삭제되었습니다.")
                        self.user_df.to_csv(path + '/user_csv',
                                            index=False, encoding='cp949')  # 변경내용 저장
                        return False
                    else:
                        print("비밀번호가 일치하지 않습니다.")
                        return False


class Born(Creater):
    def __init__(self, user_df):
        Creater.__init__(self, user_df)
        self.user_df = user_df

    def command(self):
        while True:
            print("--" * 30)
            self.command_num = input(
                "$$$ 은행 시스템입니다$$$\n-1번 : 유저 생성 \n-2번 : 유저 정보 확인 \n-3번 : 계좌전송(랜덤) \n-4번 : 유저 삭제 \n >> 원하는 서비스를 선택하십시오 : ")
            if self.command_num == "Exit" or self.command_num == "exit":
                print("시스템을 종료합니다.")
                exit()

            elif self.command_num == "1":
                a = Creater(self.user_df)

                if a.create_name() == False:
                    continue
                if a.create_id() == False:
                    continue
                if a.create_pw() == False:
                    continue
                if a.create_acc() == False:
                    continue
                if a.create_df() == False:
                    continue
                # self.user_df.to_csv(path + '/user_csv',
                #                     index=False, encoding='cp949')  # 변경내용 저장
            elif self.command_num == "2":
                print("-" * 40)
                print("### 확인 모드입니다 ###")
                if self.user_df.empty == True:
                    print("비어있음")
                else:
                    self.user_df = pd.read_csv(path + '/user_csv', sep=',',
                                               encoding='cp949')
                    print(self.user_df)
                    continue
            elif self.command_num == "3":
                print("-" * 40)
                print("### 랜덤 계좌 송금 모드입니다 ###")
                s = Remittance(self.user_df)
                s.remit()
            elif self.command_num == "4":
                b = Killer(self.user_df)
                if b.kill() == False:
                    continue
            else:
                print("다시 입력하여 주십시오")
                continue


a = Born(user_df)
a.command()
