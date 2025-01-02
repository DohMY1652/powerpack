def process_data():
    if len(sen_values) >= 9 and len(ref_values) == 6:
        # 임의의 값 2개를 ref_values에 추가
        ref_values.extend([1.0, 2.0])  # 임의의 값 (예: 1.0, 2.0)

        # 원하는 쌍의 인덱스 리스트 (예: [1, 2, 3])
        selected_pairs = [0, 1, 2, 3, 4, 5]

        # 구분선 출력
        print("-----------")
        
        # 제목 출력 (ref가 먼저 오도록 변경)
        print(f"{'ref':10} {'sen':10} {'error':10} {'micro':10} {'macro':10} {'atm':10}")

        # ref_values, sen_values, error (차이) 출력
        for i in selected_pairs:
            if i < len(sen_values) and i < len(ref_values):
                ref_value = ref_values[i]
                sen_value = sen_values[i]
                error = ref_value - sen_value  # 차이 계산

                # pwm_values 3개씩 짤라서 출력
                if len(mpc_pwm_values) >= (i + 1) * 3:  # i번째 쌍에 대해 pwm_values가 충분히 있는지 확인
                    micro = mpc_pwm_values[i * 3]     # 첫 번째 값
                    macro = mpc_pwm_values[i * 3 + 1] # 두 번째 값
                    atm = mpc_pwm_values[i * 3 + 2]   # 세 번째 값
                else:
                    micro = macro = atm = 0  # pwm_values가 부족한 경우 0으로 채우기

                # 각 값의 너비를 일정하게 맞추어 정렬하여 출력
                print(f"{ref_value:10.2f} {sen_value:10.2f} {error:10.2f} {micro:10} {macro:10} {atm:10}")

        # 구분선 출력
        print("-----------")
        
        # 6, 7 인덱스에 대한 구분선
        print("===========")

        # 제목 출력 (ref가 먼저 오도록 변경)
        print(f"{'ref':10} {'sen':10} {'error':10} {'pwm':10}")

        # /rl_pwm에서 값을 읽어와서 출력 (6번과 7번 PWM을 처리)
        if len(rl_pwm_values) == 2:
            # rl_pwm_values의 0번째 값은 6번의 PWM, 1번째 값은 7번의 PWM
            pwm_value_6 = rl_pwm_values[0]  # 6번의 PWM
            pwm_value_7 = rl_pwm_values[1]  # 7번의 PWM

            # 6번, 7번에 대한 출력 (ref, sen, error는 임시로 0으로 설정)
            ref_value_6 = 301.325 
            sen_value_6 = sen_values[0] 
            error_6 = ref_value_6 - sen_value_6  # 차이 계산 (이 부분은 실제 값에 맞게 수정 필요)

            ref_value_7 = 31.325 
            sen_value_7 = sen_values[1] 
            error_7 = ref_value_7 - sen_value_7  # 차이 계산 (이 부분은 실제 값에 맞게 수정 필요)

            # 6번과 7번에 대해 ref, sen, error, pwm 출력
            print(f"{ref_value_6:10.2f} {sen_value_6:10.2f} {error_6:10.2f} {pwm_value_6:10}")
            print(f"{ref_value_7:10.2f} {sen_value_7:10.2f} {error_7:10.2f} {pwm_value_7:10}")

        # 구분선 출력
        print("-----------")
