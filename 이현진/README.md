# 이현진 TIL

## Day1
### Dependency Injection - 의존성 주입

#### 의존성

- 클래스 간의 의존 관계를 가지는 상황
- A클래스 내부에서 B클래스가 생성, 사용되는 경우

#### 의존성 주입

- 클래스 외부에서 객체를 생성 후 넣어주는 경우
- <b>일종의 디자인 패턴!</b>

```kotlin
class MainActivity : AppCompatActivity() {
   
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        
        val preson = Person("hyunjin")
    }
}

class Person(val name: String)

// Di가 없는 코드
// class Person{
// 		val name = "hyunjin"
// }
```

- Person 클래스는 name에 의존적

#### 장점

- 코드의 재사용성이 증가한다
- 테스트에 용이하다
- 클래스간 결합도가 낮아진다
- 코드 변경에 유연하고 자유롭다
- 보일러 플레이트 코드가 줄어든다
- 앱 생명주기에 따라 관리되어 적절한 시점에 필요한 객체들이 자동으로 주입된다

### Dagger Hilt

- 인스턴스를 클래스 외부에서 주입하기 위해서는 인스턴스에 대한 생명주기 관리가 필요
- Hilt는 이를 자동으로 관리해주는 안드로이드 전용 DI 라이브러리
