from fitness_evaluator import FitnessEvaluator

def run_tests():
    evaluator = FitnessEvaluator()
    
    print("--- BẮT ĐẦU TEST FITNESS ---")
    
    # Test 1: Robot chạy đẹp (Di chuyển 1m, không va chạm, ổn định)
    # Trọng số: w_dist=1.0, w_oscillation=5.0
    # Lần 1: set last_vx=0.2
    evaluator.calculate(dist=0.0, collision_flag=0, current_vx=0.2, current_vy=0.0) 
    # Lần 2: giữ nguyên vx=0.2 (osc=0) -> fit = 1.0 * 1.0 - 0 - 0 = 1.0
    f1 = evaluator.calculate(dist=1.0, collision_flag=0, current_vx=0.2, current_vy=0.0)
    print(f"Test 1 (Di chuyển tốt): {f1:.2f}")

    # Reset evaluator cho test mới (hoặc tạo cái mới)
    evaluator = FitnessEvaluator()
    evaluator.calculate(dist=0.0, collision_flag=0, current_vx=0.0, current_vy=0.0)
    
    # Test 2: Robot va chạm (Di chuyển 0.1m, va chạm, đứng im)
    f2 = evaluator.calculate(dist=0.1, collision_flag=1, current_vx=0.0, current_vy=0.0)
    print(f"Test 2 (Va chạm): {f2:.2f}")

    # Test 3: Robot rung lắc
    evaluator = FitnessEvaluator()
    # 1. Set tốc độ cơ sở 0.2
    evaluator.calculate(dist=0.0, collision_flag=0, current_vx=0.2, current_vy=0.0)
    # 2. Thay đổi đột ngột sang -0.5 (osc = 0.7)
    f3 = evaluator.calculate(dist=0.5, collision_flag=0, current_vx=-0.5, current_vy=0.0)
    print(f"Test 3 (Rung lắc): {f3:.2f}")

    print("--- KẾT THÚC TEST ---")

if __name__ == '__main__':
    run_tests()